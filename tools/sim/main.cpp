// Runs one of the robot's autons on the desktop: prints motions as they end and the final pose,
// writes the pose log (JSON) and the path over the field map (SVG)

#include "main.h"
#include "tide/screen.hpp"
#include "world.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#if __has_include("sim.hpp")
#include "sim.hpp"
#else
inline void simRobot(sim::World&) {}
#endif

namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////
#pragma region Output //////////////////////////////////////////
////////////////////////////////////////////////////////////////

void writeJson(const fs::path& file, const std::string& auton, const sim::World& w) {
	std::ofstream out(file);
	out << "{\"auton\": \"" << auton << "\",\n";
	out << "\"robot\": {\"width\": " << w.width << ", \"front\": " << w.front << ", \"back\": " << w.back << "},\n";
	out << "\"frames\": [\n";
	for (std::size_t i = 0; i < w.frames.size(); i++) {
		const sim::Frame& f = w.frames[i];
		char line[160];
		std::snprintf(line, sizeof(line), "[%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f]%s\n",
					  f.t, f.x, f.y, f.theta, f.ox, f.oy, f.otheta, i + 1 < w.frames.size() ? "," : "");
		out << line;
	}
	out << "],\n\"events\": [\n";
	for (std::size_t i = 0; i < w.events.size(); i++)
		out << "[" << w.events[i].first << ", \"" << w.events[i].second << "\"]" << (i + 1 < w.events.size() ? "," : "") << "\n";
	out << "]}\n";
}

// SVG y points down, field +y points away from the audience
std::string points(const sim::World& w, bool odom) {
	std::string s;
	char p[32];
	for (const sim::Frame& f : w.frames) {
		std::snprintf(p, sizeof(p), "%.2f,%.2f ", odom ? f.ox : f.x, -(odom ? f.oy : f.y));
		s += p;
	}
	return s;
}

void writeSvg(const fs::path& file, const fs::path& field, const sim::World& w) {
	std::ofstream out(file);
	out << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"-79 -79 158 158\" width=\"790\" height=\"790\">\n";
	if (!field.empty())
		out << "<image href=\"" << fs::relative(field, file.parent_path()).string() << "\" x=\"-79\" y=\"-79\" width=\"158\" height=\"158\"/>\n";
	out << "<polyline fill=\"none\" stroke=\"#ff2bd6\" stroke-width=\"0.5\" stroke-dasharray=\"1.5 1\" points=\"" << points(w, true) << "\"/>\n";
	out << "<polyline fill=\"none\" stroke=\"#00a2ff\" stroke-width=\"0.7\" points=\"" << points(w, false) << "\"/>\n";

	// Footprint where it ended
	const double h = sim::toRad(w.theta);
	out << "<polygon fill=\"#00a2ff\" fill-opacity=\"0.25\" stroke=\"#00a2ff\" stroke-width=\"0.4\" points=\"";
	for (auto [r, f] : {std::pair{-w.width / 2, -w.back}, {-w.width / 2, w.front}, {w.width / 2, w.front}, {w.width / 2, -w.back}})
		out << w.x + r * std::cos(h) + f * std::sin(h) << "," << -(w.y - r * std::sin(h) + f * std::cos(h)) << " ";
	out << "\"/>\n<text x=\"-77\" y=\"-75\" font-size=\"3\" font-family=\"monospace\">blue: robot  pink: odom</text>\n</svg>\n";
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Main ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

// sim [auton] [--blue] [--time ms] [--out dir] [--field field.svg], or sim --list for auton names
int main(int argc, char** argv) {
	std::string auton, outDir = "out", field;
	bool blue = false, list = false;
	std::uint32_t limit = 15000;	// [ms] auton period
	for (int i = 1; i < argc; i++) {
		const std::string arg = argv[i];
		if (arg == "--blue") blue = true;
		else if (arg == "--list") list = true;
		else if (arg == "--time" && i + 1 < argc) limit = std::stoul(argv[++i]);
		else if (arg == "--out" && i + 1 < argc) outDir = argv[++i];
		else if (arg == "--field" && i + 1 < argc) field = argv[++i];
		else auton = arg;
	}

	sim::World& w = sim::world();
	simRobot(w);

	bool started = false, finished = false, found = true;
	pros::Task([&] {
		initialize();
		const auto& list = tide::screen::detail::autons;
		const auto it = std::find_if(list.begin(), list.end(), [&](const auto& a) { return auton == a.name; });
		if (it == list.end()) {
			found = false;
			finished = true;
			return;
		}
		tide::screen::detail::autonIdx = it - list.begin();
		tide::screen::detail::side = blue ? tide::Alliance::BLUE : tide::Alliance::RED;
		w.frames.clear();
		w.logStart = w.sched.now;
		started = true;
		autonomous();
		finished = true;
	});
	w.sched.run([&] { return finished || (started && w.sched.now - w.logStart >= limit); }, [&] { w.step(); });

	if (list) {
		for (const auto& a : tide::screen::detail::autons) std::printf("%s\n", a.name);
		std::fflush(stdout);
		std::_Exit(0);
	}
	if (!found) {
		std::printf("Pick an auton:");
		for (const auto& a : tide::screen::detail::autons) std::printf(" \"%s\"", a.name);
		std::printf("\n");
		std::fflush(stdout);
		std::_Exit(1);
	}

	const double t = (w.sched.now - w.logStart) / 1000.0;
	if (!finished) std::printf("%6.2f  auton still running when time ran out\n", t);
	std::printf("\nFinal pose  x %.2f  y %.2f  heading %.1f\n", w.x, w.y, w.theta);
	std::printf("Odom        x %.2f  y %.2f  heading %.1f\n", w.odomX, w.odomY, w.odomTheta);

	fs::create_directories(outDir);
	const fs::path base = fs::path(outDir) / auton;
	writeJson(base.string() + ".json", auton, w);
	writeSvg(base.string() + ".svg", field.empty() ? fs::path() : fs::absolute(field), w);
	std::printf("Wrote %s.json and %s.svg\n", base.c_str(), base.c_str());

	std::fflush(stdout);
	std::_Exit(0);	// Background tasks never return
}

#pragma endregion
////////////////////////////////////////////////////////////////
