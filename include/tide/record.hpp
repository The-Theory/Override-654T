////////////////////////////////////////////////////////////////
// Tide - Controller binding library for PROS				  //
// Built for 654T - Tsunami, VEX Override 2026-2027			  //
//															  //
// By Theo Hallgren											  //
// Documentation in docs/tide.md							  //
//															  //
// Source available on: github.com/The-Theory/Override-654T	  //
// under GPL-3.0 license									  //
////////////////////////////////////////////////////////////////


#pragma once

#include "tide/odom.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Driver runs recorded as where the robot was, what its distance sensors read and what its mechanisms
// did, then played back as an auton: the drive follows the path, correcting itself so the sensors read
// what they read in the recording, and the mechanisms repeat on cue. A recording replays straight away
// from memory, goes on the SD card if there is one, and send() hands it to tools/recording.py to keep
// in the source

namespace tide {

// SD card folder; optional on the brain. The desktop sim points it somewhere on disk
inline std::string sdRoot = "/usd/";

////////////////////////////////////////////////////////////////
#pragma region Recorder ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// A mechanism a recording keeps besides the drive, like a claw or lift
struct Channel {
	const char* name;
	std::function<double()> read;
	std::function<void(double)> write;	// called on playback whenever the recorded value changes
	double change = 0.5;				// smaller changes aren't replayed, so sensor noise doesn't
};

class Recorder {
public:
	Recorder(Odom& odom, lemlib::Chassis& chassis, std::vector<Channel> channels)
		: odom(odom), chassis(chassis), channels(std::move(channels)) {}

	bool recording() const { return active; }

	// A recording built into the program, as tools/recording.py saved it; replays when none is newer
	void builtIn(const std::string& name, const char* text) { if (*text) compiled[name] = text; }

	// Samples every PERIOD_MS in the background from the pose odom has now until stop()
	void start(const std::string& name) {
		if (active || sampling) return;
		file = name;
		rows.clear();
		active   = true;
		sampling = true;
		pros::Task([this] {
			const std::uint32_t t0 = pros::millis();
			std::uint32_t wake = t0;
			while (active && pros::millis() - t0 < MAX_MS) {
				const lemlib::Pose pose = chassis.getPose();
				rows.push_back((pros::millis() - t0) / 1000.0);
				for (double v : {double(pose.x), double(pose.y), double(pose.theta)}) rows.push_back(v);
				for (const Channel& c : channels) rows.push_back(c.read());
				for (const DistanceMount& m : odom.getMounts()) rows.push_back(Odom::inches(m.sensor));
				pros::Task::delay_until(&wake, PERIOD_MS);
			}
			sampling = false;
		});
		std::printf("Recording %s\n", name.c_str());
	}

	// Ends the recording and keeps it for replay until the program restarts; send() keeps it longer
	void stop() {
		if (!active) return;
		active = false;
		while (sampling) pros::delay(5);

		std::string text = "t x y theta";
		for (const Channel& c : channels) text += std::string(" ") + c.name;
		for (std::size_t k = 0; k < odom.getMounts().size(); k++) text += " d" + std::to_string(k + 1);
		text += "\n";
		char number[24];
		for (std::size_t i = 0; i < rows.size(); i++) {
			std::snprintf(number, sizeof(number), (i + 1) % width() ? "%.5g " : "%.5g\n", rows[i]);
			text += number;
		}
		const std::size_t count = rows.size() / width();
		kept[file] = text;
		std::printf("Recorded %s: %.1f s\n", file.c_str(), count * PERIOD_MS / 1000.0);

		if (std::FILE* f = std::fopen(path(file).c_str(), "w")) {	// Fails at once with no card
			std::fputs(text.c_str(), f);
			std::fclose(f);
		}
	}

	// Prints the newest copy of a recording over the terminal for tools/recording.py to save into the
	// source; paced so the radio link to the controller keeps up. False if there's none
	bool send(const std::string& name) const {
		const std::string all = text(name);
		if (all.empty()) return false;
		std::printf("=== recording %s %zu rows ===\n", name.c_str(), std::size_t(std::count(all.begin(), all.end(), '\n')) - 1);
		std::size_t line = 0;
		for (std::size_t from = 0, to; from < all.size(); from = to + 1) {
			to = all.find('\n', from);
			std::printf("%.*s\n", int(to - from), all.c_str() + from);
			if (++line % 10 == 0) pros::delay(40);
		}
		std::printf("=== end %s ===\n", name.c_str());
		return true;
	}

	/**
	 * Plays a recording back from where it started: sets the pose there, follows the path,
	 * and repeats each mechanism change as the path reaches it. No wall resets on the way: the
	 * recording's own odom drifted the same way, so resets would pull the two apart.
	 * False if it's missing or was recorded with other channels.
	 */
	bool replay(const std::string& name) {
		std::vector<Waypoint> path;
		std::vector<std::vector<double>> values;
		if (!load(name, path, values)) return false;

		chassis.setPose(path[0].x, path[0].y, path[0].theta);
		std::vector<double> last(channels.size(), NAN);
		odom.track(path, [&](std::size_t i) {
			match(path[i], values[i].data() + channels.size());
			for (std::size_t c = 0; c < channels.size(); c++) {
				if (!std::isnan(last[c]) && std::fabs(values[i][c] - last[c]) < channels[c].change) continue;
				channels[c].write(values[i][c]);
				last[c] = values[i][c];
			}
		});
		return true;
	}

private:
	static constexpr std::uint32_t MAX_MS    = 75000;	// Skills plus margin, so a forgotten recording stops
	static constexpr std::uint32_t PERIOD_MS = 20;		// Sample spacing; shorter makes the terminal dump longer
	static constexpr int SPAN = 2;					// samples either side a speed is measured over, against odom jitter

	Odom& odom;
	lemlib::Chassis& chassis;
	const std::vector<Channel> channels;
	std::string file;
	std::vector<double> rows;	// t, x, y, theta, then channels, per sample
	std::map<std::string, std::string> kept;		// Recorded since the program started
	std::map<std::string, const char*> compiled;	// Built in
	volatile bool active = false, sampling = false;

	std::size_t width() const { return 4 + channels.size() + odom.getMounts().size(); }

	static constexpr double MATCH_RANGE = 40;	// [in] past this a distance sensor's ~5% error outweighs what it fixes
	static constexpr double MATCH_GATE  = 4;	// [in] off by more and it's seeing something else (a moved piece, a robot)
	static constexpr double MATCH_NEAR  = 4;	// [in] only while the robot is near where the recording was
	static constexpr double MATCH_TURN  = 20;	// [deg/s] faster turns sweep the beam faster than the sensor updates
	static constexpr double MATCH_ANGLE = 3;	// [deg] off the recording's heading, an angled beam reads too differently
	static constexpr double MATCH_GAIN  = 0.1;	// of each sample's mismatch moved per sample, so one bad reading barely counts; 0 turns matching off
	static constexpr double MATCH_STEP  = 0.1;	// [in] most odom moves per sample

	/**
	 * Nudges odom so each distance sensor reads what it read at this point of the recording. The
	 * target is the recording, not the field map, so it can't pull replay away from the frame the
	 * recording's odom drifted in. Sensor lag is the same in both runs at the same speed, so it cancels.
	 */
	void match(const Waypoint& ref, const double* recorded) {
		const lemlib::Pose pose = chassis.getPose();
		if (std::hypot(pose.x - ref.x, pose.y - ref.y) > MATCH_NEAR || std::fabs(ref.w) > MATCH_TURN ||
			std::fabs(std::remainder(pose.theta - ref.theta, 360.0)) > MATCH_ANGLE) return;
		double dx = 0, dy = 0;
		const std::vector<DistanceMount>& mounts = odom.getMounts();
		for (std::size_t k = 0; k < mounts.size(); k++) {
			const DistanceMount& m = mounts[k];
			const double was = recorded[k], now = Odom::inches(m.sensor);
			if (std::isnan(was) || std::isnan(now) || was > MATCH_RANGE) continue;
			// What it should read from where odom puts it now, which trails or leads the recording's spot
			const Point beam  = toField(0, 1, ref.theta + m.facing);
			const Point there = toField(m.right, m.forward, ref.theta), here = toField(m.right, m.forward, pose.theta);
			const double expected = was + (ref.x + there.x - pose.x - here.x) * beam.x + (ref.y + there.y - pose.y - here.y) * beam.y;
			const double off = now - expected;	// Longer: the robot is further back along the beam than odom says
			if (std::fabs(off) > MATCH_GATE) continue;
			dx -= off * beam.x * MATCH_GAIN;
			dy -= off * beam.y * MATCH_GAIN;
		}
		const double step = std::hypot(dx, dy);
		if (step == 0) return;
		const double scale = std::min(1.0, MATCH_STEP / step);
		chassis.setPose(pose.x + dx * scale, pose.y + dy * scale, pose.theta);
	}
	static std::string path(const std::string& name) { return sdRoot + name + ".txt"; }

	// Newest first: recorded this run, then the SD card, then built in
	std::string text(const std::string& name) const {
		if (kept.count(name)) return kept.at(name);
		if (std::FILE* f = std::fopen(path(name).c_str(), "r")) {
			std::string out;
			char chunk[512];
			for (std::size_t n; (n = std::fread(chunk, 1, sizeof(chunk), f)) > 0;) out.append(chunk, n);
			std::fclose(f);
			return out;
		}
		return compiled.count(name) ? compiled.at(name) : "";
	}

	bool load(const std::string& name, std::vector<Waypoint>& path, std::vector<std::vector<double>>& values) const {
		const std::string all = text(name);
		if (all.empty()) {
			std::printf("Replay %s FAILED: nothing recorded\n", name.c_str());
			return false;
		}
		const std::size_t headerEnd = all.find('\n');
		const std::size_t columns = std::count(all.begin(), all.begin() + std::min(headerEnd, all.size()), ' ') + 1;
		if (columns != width()) {
			std::printf("Replay %s FAILED: recorded with other mechanisms\n", name.c_str());
			return false;
		}
		std::vector<double> row(width());
		const char* at = all.c_str() + headerEnd;
		auto next = [&] {
			for (double& v : row) {
				char* end;
				v = std::strtod(at, &end);
				if (end == at) return false;
				at = end;
			}
			return true;
		};
		while (next()) {
			path.push_back({row[0], row[1], row[2], row[3]});
			values.emplace_back(row.begin() + 4, row.end());
		}
		if (path.size() < 2 * SPAN + 1) {
			std::printf("Replay %s FAILED: too short\n", name.c_str());
			return false;
		}
		speeds(path);
		return true;
	}

	// Speeds and accelerations from positions, each over a window so one jittery sample doesn't spike them
	static void speeds(std::vector<Waypoint>& path) {
		const std::size_t n = path.size();
		auto at = [n](std::ptrdiff_t i) { return std::size_t(std::clamp<std::ptrdiff_t>(i, 0, n - 1)); };
		for (std::size_t i = 0; i < n; i++) {
			const Waypoint &a = path[at(i - SPAN)], &b = path[at(i + SPAN)];
			const double dt = b.t - a.t, h = path[i].theta * std::numbers::pi / 180;
			path[i].v = ((b.x - a.x) * std::sin(h) + (b.y - a.y) * std::cos(h)) / dt;
			path[i].w = std::remainder(b.theta - a.theta, 360.0) / dt;
		}
		std::vector<double> a(n), alpha(n);
		for (std::size_t i = 0; i < n; i++) {
			const Waypoint &p = path[at(i - SPAN)], &q = path[at(i + SPAN)];
			a[i]     = (q.v - p.v) / (q.t - p.t);
			alpha[i] = (q.w - p.w) / (q.t - p.t);
		}
		for (std::size_t i = 0; i < n; i++) path[i].a = a[i], path[i].alpha = alpha[i];
	}
};

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace tide
