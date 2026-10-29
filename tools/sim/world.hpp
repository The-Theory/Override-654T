#pragma once

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

// Off-brain stand-in for the robot and field: fake PROS devices read and write this, physics moves it
namespace sim {

////////////////////////////////////////////////////////////////
#pragma region Scheduler ///////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * PROS tasks on sim time. Every task gets a thread, but only the one holding the baton runs,
 * and time only moves once every task is waiting in delay(), so runs are deterministic.
 * ponytail: threads; the wasm build swaps in Emscripten fibers
 */
class Scheduler {
public:
	std::uint32_t now = 0;	// [ms]

	void spawn(std::function<void()> fn) {
		auto task = std::make_unique<Task>();
		Task* t = task.get();
		t->wake = now;
		tasks.push_back(std::move(task));
		std::thread([this, t, fn] {
			{
				std::unique_lock lock(m);
				t->cv.wait(lock, [&] { return current == t; });
			}
			fn();
			std::lock_guard lock(m);
			t->finished = true;
			current = nullptr;
			mainCv.notify_one();
		}).detach();	// Background loops never end; the process exits around them
	}

	// From inside a task
	void delay(std::uint32_t ms) {
		std::unique_lock lock(m);
		Task* self = current;
		self->wake = now + ms;
		current = nullptr;
		mainCv.notify_one();
		self->cv.wait(lock, [&] { return current == self; });
	}

	// Runs tasks until done(), stepping the world 1 ms at a time between them
	void run(const std::function<bool()>& done, const std::function<void()>& step) {
		std::unique_lock lock(m);
		while (!done()) {
			Task* next = nullptr;
			for (auto& t : tasks)
				if (!t->finished && (!next || t->wake < next->wake)) next = t.get();
			if (!next) return;
			while (now < next->wake && !done()) {
				step();
				now++;
			}
			if (done()) return;
			current = next;
			next->cv.notify_one();
			mainCv.wait(lock, [&] { return current == nullptr; });
		}
	}

private:
	struct Task {
		std::uint32_t wake = 0;
		bool finished = false;
		std::condition_variable cv;
	};

	std::mutex m;
	std::condition_variable mainCv;
	std::vector<std::unique_ptr<Task>> tasks;
	Task* current = nullptr;
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Devices /////////////////////////////////////////
////////////////////////////////////////////////////////////////

enum Brake { COAST, BRAKE, HOLD };

struct Motor {
	int    mv        = 0;		// commanded, as the robot code sees it
	Brake  brake     = COAST;
	double cartridge = 200;		// free speed [rpm]
	double rpm       = 0;
	double degrees   = 0;
};

struct Rotation {
	double centideg = 0;
	double velocity = 0;		// [centideg/s]
};

// Where a distance sensor really sits, from the tracking center
struct Mount {
	double right, forward;		// [in]
	double facing;				// [deg] clockwise from robot front
};

// A motor-driven arm or lift read by a rotation sensor
struct Joint {
	std::vector<std::int8_t> ports;
	int    sensor;
	double sensorPerMotor = 1;		// sensor degrees per motor degree
	double gravityMv      = 0;		// voltage gravity takes back while lifted
	double min            = -INFINITY;	// hard stops [sensor deg]
	double max            = INFINITY;
	double angle          = 0;
	double velocity       = 0;		// [sensor deg/s]
};

struct Frame {
	double t, x, y, theta;			// truth
	double ox, oy, otheta;			// what odom thinks
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region World ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

inline double toRad(double degrees) { return degrees * std::numbers::pi / 180; }

class World {
public:
	Scheduler sched;
	std::map<int, Motor> motors;
	std::map<int, Rotation> rotations;
	std::map<int, Mount> mounts;
	std::vector<Joint> joints;

	// Robot truth; set from the robot's own config, override in src/sim.hpp
	double halfField = 70.2;		// center to inside of the perimeter [in]
	double width = 18;				// footprint [in]
	double front = 9, back = 9;		// [in] from the tracking center
	double driveTau = 0.25;			// [s] drive speed time constant; lower is snappier
	double trackWidth = 12, wheelDiameter = 3.25, wheelRpm = 450;
	std::vector<std::int8_t> left, right;
	int    trackingPort = 0;
	double trackingDiameter = 2, trackingOffset = 0;	// [in] + right of center
	double imuDrift = 0;			// [deg/s]

	// Pose: LemLib frame, heading clockwise from +y [deg]
	double x = 0, y = 0, theta = 0;
	double odomX = 0, odomY = 0, odomTheta = 0;	// written by the fake chassis, for the log
	double imuOffset = 0;
	bool   placed = false;

	std::vector<Frame> frames;
	std::vector<std::pair<double, std::string>> events;
	std::uint32_t logStart = 0;

	Motor& motor(int port) { return motors[std::abs(port)]; }
	Rotation& rotation(int port) { return rotations[std::abs(port)]; }

	// First setPose puts the robot there, like setting it on the tiles; later ones only move odom
	void place(double px, double py, double heading) {
		if (placed) return;
		placed = true;
		imuOffset -= heading - theta;	// IMU reading doesn't jump
		x = px;
		y = py;
		theta = heading;
	}

	double imu() const { return theta + imuOffset + imuDrift * sched.now / 1000.0; }

	void event(const char* fmt, ...) {
		char text[160];
		va_list args;
		va_start(args, fmt);
		std::vsnprintf(text, sizeof(text), fmt, args);
		va_end(args);
		const double t = (sched.now - logStart) / 1000.0;
		std::printf("%6.2f  %s\n", t, text);
		events.push_back({t, text});
	}

	// Distance sensor reading [mm]; 9999 like the real one with nothing in range
	std::int32_t distance(int port) {
		const auto it = mounts.find(std::abs(port));
		if (it == mounts.end()) return 9999;
		const Mount& m = it->second;
		const double h = toRad(theta);
		const double sx = x + m.right * std::cos(h) + m.forward * std::sin(h);
		const double sy = y - m.right * std::sin(h) + m.forward * std::cos(h);
		const double beam = toRad(theta + m.facing);
		const double dx = std::sin(beam), dy = std::cos(beam);
		double hit = INFINITY;
		if (dx > 1e-9)  hit = std::min(hit, ( halfField - sx) / dx);
		if (dx < -1e-9) hit = std::min(hit, (-halfField - sx) / dx);
		if (dy > 1e-9)  hit = std::min(hit, ( halfField - sy) / dy);
		if (dy < -1e-9) hit = std::min(hit, (-halfField - sy) / dy);
		const double mm = hit * 25.4;
		return mm > 2000 ? 9999 : std::lround(mm);
	}

	// One millisecond of physics
	void step() {
		constexpr double DT = 0.001;
		const double inPerSec = wheelRpm / 60 * std::numbers::pi * wheelDiameter;

		const double h0 = theta;
		const double x0 = x, y0 = y;
		const double vl = sideSpeed(left, vL, inPerSec, DT);
		const double vr = sideSpeed(right, vR, inPerSec, DT);
		const double omega = (vl - vr) / trackWidth;	// [rad/s] clockwise
		theta += omega * DT * 180 / std::numbers::pi;
		const double mid = toRad((h0 + theta) / 2);
		const double v = (vl + vr) / 2;
		x += v * std::sin(mid) * DT;
		y += v * std::cos(mid) * DT;
		hitWalls();

		// Wheels can't outrun a wall, so what it let through sets their speed; that's what makes a stall
		const double forward = (x - x0) * std::sin(mid) + (y - y0) * std::cos(mid);
		const double dTheta = toRad(theta - h0);
		vL = forward / DT + omega * trackWidth / 2;
		vR = forward / DT - omega * trackWidth / 2;
		driveMotors(left, vL);
		driveMotors(right, vR);

		if (trackingPort) {
			Rotation& r = rotation(trackingPort);
			const double travel = forward - trackingOffset * dTheta;
			const double delta = travel / (std::numbers::pi * trackingDiameter) * 36000;
			r.centideg += delta;
			r.velocity = delta / DT;
		}

		for (Joint& j : joints) stepJoint(j, DT);
		for (auto& [port, m] : motors)
			if (!driven(port)) freeSpin(m, DT);

		if ((sched.now - logStart) % 10 == 0)
			frames.push_back({(sched.now - logStart) / 1000.0, x, y, theta, odomX, odomY, odomTheta});
	}

private:
	double vL = 0, vR = 0;	// [in/s]

	static double tau(Brake b, int mv, double running) {
		if (mv != 0) return running;
		return b == COAST ? 0.3 : b == BRAKE ? 0.08 : 0.04;	// Coast rolls on, brake shorts the motor
	}

	double sideSpeed(const std::vector<std::int8_t>& ports, double v, double maxSpeed, double dt) {
		if (ports.empty()) return 0;
		double mv = 0;
		for (int p : ports) mv += motor(p).mv;
		mv /= ports.size();
		const double target = mv / 12000 * maxSpeed;
		return v + (target - v) * dt / tau(motor(ports[0]).brake, mv, driveTau);
	}

	void driveMotors(const std::vector<std::int8_t>& ports, double v) {
		for (int p : ports) {
			Motor& m = motor(p);
			m.rpm = v / (std::numbers::pi * wheelDiameter) * 60 * m.cartridge / wheelRpm;
			m.degrees += m.rpm * 6 * 0.001;
		}
	}

	// ponytail: pushes the center straight out of each wall; no sliding or turning against it
	void hitWalls() {
		const double h = toRad(theta);
		double minX = INFINITY, maxX = -INFINITY, minY = INFINITY, maxY = -INFINITY;
		for (double r : {-width / 2, width / 2})
			for (double f : {-back, front}) {
				const double cx = x + r * std::cos(h) + f * std::sin(h);
				const double cy = y - r * std::sin(h) + f * std::cos(h);
				minX = std::min(minX, cx); maxX = std::max(maxX, cx);
				minY = std::min(minY, cy); maxY = std::max(maxY, cy);
			}
		if (maxX > halfField)  x -= maxX - halfField;
		if (minX < -halfField) x += -halfField - minX;
		if (maxY > halfField)  y -= maxY - halfField;
		if (minY < -halfField) y += -halfField - minY;
	}

	bool driven(int port) const {
		auto in = [port](const std::vector<std::int8_t>& ports) {
			return std::any_of(ports.begin(), ports.end(), [port](int p) { return std::abs(p) == port; });
		};
		if (in(left) || in(right)) return true;
		return std::any_of(joints.begin(), joints.end(), [&](const Joint& j) { return in(j.ports); });
	}

	static void freeSpin(Motor& m, double dt) {
		const double target = m.mv / 12000.0 * m.cartridge;
		m.rpm += (target - m.rpm) * dt / tau(m.brake, m.mv, 0.05);
		m.degrees += m.rpm * 6 * dt;
	}

	void stepJoint(Joint& j, double dt) {
		double mv = 0;
		for (int p : j.ports) mv += motor(p).mv;
		mv /= j.ports.size();
		const Brake brake = motor(j.ports[0]).brake;
		const bool held = mv == 0 && brake != COAST;
		const double freeSpeed = motor(j.ports[0]).cartridge * 6 * j.sensorPerMotor;	// [sensor deg/s]
		const double target = held ? 0 : (mv - j.gravityMv) / 12000 * freeSpeed;
		j.velocity += (target - j.velocity) * dt / tau(brake, mv, 0.05);
		j.angle += j.velocity * dt;
		if (j.angle < j.min) { j.angle = j.min; j.velocity = std::max(j.velocity, 0.0); }
		if (j.angle > j.max) { j.angle = j.max; j.velocity = std::min(j.velocity, 0.0); }

		Rotation& r = rotation(j.sensor);
		r.centideg = j.angle * 100;
		r.velocity = j.velocity * 100;
		for (int p : j.ports) {
			Motor& m = motor(p);
			m.rpm = j.velocity / 6 / j.sensorPerMotor;
			m.degrees = j.angle / j.sensorPerMotor;
		}
	}
};

// Function-local so fake devices built in static initializers find it ready
inline World& world() {
	static World w;
	return w;
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace sim
