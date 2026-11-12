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
#include <functional>
#include <string>
#include <vector>

// Driver runs recorded to the SD card as where the robot was and what its mechanisms did,
// then played back as an auton: the drive follows the path, the mechanisms repeat on cue

namespace tide {

// SD card folder; the desktop sim points it somewhere on disk
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

	// Samples every 10 ms in the background from the pose odom has now until stop()
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
				pros::Task::delay_until(&wake, 10);
			}
			sampling = false;
		});
		std::printf("Recording %s\n", name.c_str());
	}

	// Ends the recording and saves it; false if the SD card wouldn't take it
	bool stop() {
		if (!active) return false;
		active = false;
		while (sampling) pros::delay(5);

		std::FILE* f = std::fopen(path(file).c_str(), "w");
		if (!f) {
			std::printf("Recording %s NOT SAVED: no SD card?\n", file.c_str());
			return false;
		}
		std::fprintf(f, "t x y theta");
		for (const Channel& c : channels) std::fprintf(f, " %s", c.name);
		std::fprintf(f, "\n");
		for (std::size_t i = 0; i < rows.size(); i++) std::fprintf(f, (i + 1) % width() ? "%.3f " : "%.3f\n", rows[i]);
		std::fclose(f);
		std::printf("Recorded %s: %.1f s\n", file.c_str(), rows.size() / width() / 100.0);
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
			for (std::size_t c = 0; c < channels.size(); c++) {
				if (!std::isnan(last[c]) && std::fabs(values[i][c] - last[c]) < channels[c].change) continue;
				channels[c].write(values[i][c]);
				last[c] = values[i][c];
			}
		});
		return true;
	}

private:
	static constexpr std::uint32_t MAX_MS = 75000;	// Skills plus margin, so a forgotten recording stops
	static constexpr int SPAN = 5;					// samples either side a speed is measured over, against odom jitter

	Odom& odom;
	lemlib::Chassis& chassis;
	const std::vector<Channel> channels;
	std::string file;
	std::vector<double> rows;	// t, x, y, theta, then channels, per sample
	volatile bool active = false, sampling = false;

	std::size_t width() const { return 4 + channels.size(); }
	static std::string path(const std::string& name) { return sdRoot + name + ".txt"; }

	bool load(const std::string& name, std::vector<Waypoint>& path, std::vector<std::vector<double>>& values) const {
		std::FILE* f = std::fopen(Recorder::path(name).c_str(), "r");
		if (!f) {
			std::printf("Replay %s FAILED: no recording on the SD card\n", name.c_str());
			return false;
		}
		char header[256];
		const bool read = std::fgets(header, sizeof(header), f);
		std::size_t columns = 0;
		for (const char* p = header; read && *p; p++) columns += *p == ' ';
		if (columns + 1 != width()) {
			std::printf("Replay %s FAILED: recorded with other mechanisms\n", name.c_str());
			std::fclose(f);
			return false;
		}
		std::vector<double> row(width());
		auto next = [&] { return std::all_of(row.begin(), row.end(), [f](double& v) { return std::fscanf(f, "%lf", &v) == 1; }); };
		while (next()) {
			path.push_back({row[0], row[1], row[2], row[3]});
			values.emplace_back(row.begin() + 4, row.end());
		}
		std::fclose(f);
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
