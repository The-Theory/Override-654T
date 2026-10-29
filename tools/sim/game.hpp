#pragma once

#include "tide/override.hpp"
#include "world.hpp"
#include <array>
#include <cmath>
#include <functional>
#include <ostream>
#include <vector>

/**
 * VEX Override 2026-27 on the sim field: Pins, Cups, Goals, Toggles and the score.
 * Swap this file out for next season's game; the rest of the sim doesn't know about it.
 * Bookkeeping, not rigid bodies: pieces are circles the robot shoves aside, picks up and stacks.
 */
namespace sim::game {

////////////////////////////////////////////////////////////////
#pragma region Pieces //////////////////////////////////////////
////////////////////////////////////////////////////////////////

enum Color { YELLOW, RED, BLUE };
enum GoalKind { RED_GOAL, BLUE_GOAL, NEUTRAL, MIDFIELD };

constexpr double PIN_R  = 1.55;	// [in] flange radius
constexpr double CUP_R  = 1.58;
constexpr double GOAL_R = 2.5;	// [in] how close to a Goal's center a release has to be to stack there

struct Piece {
	bool   cup   = false;
	Color  top   = YELLOW, bottom = YELLOW;	// Pin halves; top is the one pointing up
	bool   lying = false;					// Cross Pins lie flat and can only be pushed
	double angle = 0;						// [deg] heading a lying Pin's top half points along
	double x = 0, y = 0;
	int    goal  = -1;						// index into goals while stacked
	bool   held  = false;

	double radius() const { return cup ? CUP_R : PIN_R; }
	bool free() const { return goal < 0 && !held; }
};

struct Goal {
	double   x, y;
	GoalKind kind;
	int      toggle;	// whose Toggle owns its yellow halves; -1 for Midfield
	std::vector<int> stack;	// piece indices, bottom first
};

struct Toggle {
	double x, y;
	double inward;		// [deg] heading from the wall into the field
	int    face = 0;	// Color showing, starts yellow
	double turn = 0;	// progress toward the next face, -1..1
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Robot ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

// What a robot does to game pieces; each team fills this in its src/sim.hpp
struct Robot {
	// Claw: picks up a standing piece (or a Pin in a Cup) at its tool point, drops it or stacks it on a Goal
	struct {
		double right = 0, forward = 0;	// tool point from the tracking center [in]
		double reach = 3;				// [in]
		std::function<bool()> grab    = [] { return false; };
		std::function<bool()> release = [] { return false; };
		std::function<bool()> lowered = [] { return true; };	// low enough to pick off the tiles
	} claw;

	// Wheel that spins a Toggle it's pressed against, with a color sensor watching the face
	struct {
		int    motor   = 0;
		int    optical = 0;
		double right = 0, forward = 0;	// contact point [in]
		double facesPerSecond = 3;		// at 12 V
	} spinner;

	std::vector<Piece> preload;			// starts in the claw
};

inline Robot robot;

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Field ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Field position of a robot-frame point (+right, +forward) on the true robot
inline tide::Point onRobot(const World& w, double right, double forward) {
	const tide::Point p = tide::toField(right, forward, w.theta);
	return {w.x + p.x, w.y + p.y};
}

class Field {
public:
	std::vector<Piece>  pieces;
	std::vector<Goal>   goals;
	std::vector<Toggle> toggles;

	// Starting layout from the manual's Appendix A, via tide::override's red-side spots rotated for blue
	void setup() {
		using namespace tide::override;
		auto flip = [](tide::Point p) { return tide::Point{-p.x, -p.y}; };

		for (tide::Point t : {toggle::Q1, toggle::Q2, flip(toggle::Q1), flip(toggle::Q2)})
			toggles.push_back({t.x, t.y, tide::bearing(t, {0, 0})});

		addGoal(goal::Q1_ALLIANCE, RED_GOAL, 0);
		addGoal(goal::Q1_NEUTRAL, NEUTRAL, 0);
		addGoal(goal::Q2_ALLIANCE, RED_GOAL, 1);
		addGoal(goal::Q2_NEUTRAL, NEUTRAL, 1);
		addGoal(flip(goal::Q1_ALLIANCE), BLUE_GOAL, 2);
		addGoal(flip(goal::Q1_NEUTRAL), NEUTRAL, 2);
		addGoal(flip(goal::Q2_ALLIANCE), BLUE_GOAL, 3);
		addGoal(flip(goal::Q2_NEUTRAL), NEUTRAL, 3);
		addGoal(goal::MIDFIELD, MIDFIELD, -1);
		for (Goal& g : goals)
			if (g.kind == NEUTRAL || g.kind == MIDFIELD) stackOn(g, add({.x = g.x, .y = g.y}));

		// Cup + Pin spots; wall ones have a lone Cup either side along the wall
		for (tide::Point p : {cuppin::RED_WALL_S, cuppin::RED_WALL_N, cuppin::AUDIENCE_WALL_W, cuppin::AUDIENCE_WALL_E,
							  cuppin::DIAGONAL_INNER, cuppin::DIAGONAL_OUTER})
			for (tide::Point at : {p, flip(p)}) {
				add({.cup = true, .x = at.x, .y = at.y});
				add({.x = at.x, .y = at.y});
				if (std::fabs(at.x) < 60 && std::fabs(at.y) < 60) continue;
				const bool alongY = std::fabs(at.x) > 60;
				for (double side : {-3.16, 3.16})
					add({.cup = true, .x = at.x + (alongY ? 0 : side), .y = at.y + (alongY ? side : 0)});
			}
		// Midfield vertices: each side's own color up
		for (tide::Point p : {cuppin::MIDFIELD_W, cuppin::MIDFIELD_S})
			for (bool blue : {false, true}) {
				const tide::Point at = blue ? flip(p) : p;
				add({.cup = true, .x = at.x, .y = at.y});
				add({.top = blue ? BLUE : RED, .bottom = blue ? RED : BLUE, .x = at.x, .y = at.y});
			}
		// Crosses on the Auton Line: a Cup with four Pins lying around it, yellow ends in
		for (tide::Point c : {cross::Q1_INNER, cross::Q1_OUTER, cross::Q2_INNER, cross::Q2_OUTER}) {
			add({.cup = true, .x = c.x, .y = c.y});
			for (double heading : {0.0, 90.0, 180.0, 270.0}) {
				const bool blue = heading < 135;	// arms pointing +x/+y face the blue side
				const tide::Point arm = tide::toField(0, 4.8, heading);
				add({.top = blue ? BLUE : RED, .bottom = YELLOW, .lying = true, .angle = heading, .x = c.x + arm.x, .y = c.y + arm.y});
			}
		}

		for (Piece p : robot.preload) {
			p.held = true;
			held.push_back(add(p));
		}
	}

	void step(World& w, double dt) {
		if (!w.placed) return;	// Nothing touches the field until the robot is set down
		moveHeld(w);
		claw(w);
		spin(w, dt);
		push(w);
		if ((w.sched.now - w.logStart) % 10 == 0) record(w);
	}

	// Hue the robot's color sensor reads off a Toggle it's touching, and how close it is
	std::pair<double, std::int32_t> optical(const World& w, int port) const {
		if (port != robot.spinner.optical) return {0, 0};
		const int t = touching(w);
		if (t < 0) return {0, 0};
		constexpr double HUE[] = {55, 0, 220};
		return {HUE[toggles[t].face], std::lround(std::clamp(255 * (1 - gap(w, toggles[t]) / 4), 0.0, 255.0))};
	}

	// Red and blue points as things stand. ponytail: every stacked Pin shows only its top half,
	// close to the manual's count; Cup orientation and match-load Pins aren't tracked
	std::pair<int, int> score(const World& w) const {
		int red = 0, blue = 0;
		const bool robotInMidfield = std::fabs(w.x) + std::fabs(w.y) < 22;	// The sim only has our red robot
		for (const Goal& g : goals)
			for (int i : g.stack) {
				const Piece& p = pieces[i];
				if (p.cup) continue;
				if (p.top == RED) red += 5;
				else if (p.top == BLUE) blue += 5;
				else {
					const int owner = g.toggle < 0 ? (robotInMidfield ? RED : YELLOW) : toggles[g.toggle].face;
					if (owner == RED) red += 10;
					if (owner == BLUE) blue += 10;
				}
			}
		return {red, blue};
	}

	void restartLog() {
		tracks.assign(pieces.size(), {});
		faces.assign(toggles.size(), {});
		scores.clear();
	}

	void writeJson(std::ostream& out) const {
		out << "\"goals\": [";
		for (std::size_t i = 0; i < goals.size(); i++)
			out << (i ? "," : "") << "[" << goals[i].x << "," << goals[i].y << "," << goals[i].kind << "]";
		out << "],\n\"toggles\": [\n";
		for (std::size_t i = 0; i < toggles.size(); i++) {
			out << (i ? ",\n" : "") << "{\"x\": " << toggles[i].x << ", \"y\": " << toggles[i].y << ", \"faces\": [";
			for (std::size_t k = 0; k < faces[i].size(); k++) out << (k ? "," : "") << "[" << faces[i][k].first << "," << faces[i][k].second << "]";
			out << "]}";
		}
		out << "],\n\"pieces\": [\n";
		for (std::size_t i = 0; i < pieces.size(); i++) {
			const Piece& p = pieces[i];
			out << (i ? ",\n" : "") << "{\"cup\": " << p.cup << ", \"top\": " << p.top << ", \"bottom\": " << p.bottom
				<< ", \"lying\": " << p.lying << ", \"angle\": " << p.angle << ", \"track\": [";
			for (std::size_t k = 0; k < tracks[i].size(); k++) {
				const Sample& s = tracks[i][k];
				char row[64];
				std::snprintf(row, sizeof(row), "%s[%.2f,%.2f,%.2f,%d]", k ? "," : "", s.t, s.x, s.y, s.where);
				out << row;
			}
			out << "]}";
		}
		out << "],\n\"score\": [";
		for (std::size_t i = 0; i < scores.size(); i++)
			out << (i ? "," : "") << "[" << scores[i][0] << "," << scores[i][1] << "," << scores[i][2] << "]";
		out << "],\n";
	}

private:
	std::vector<int> held;

	// where: 0 tiles, 1 in the claw, 2 + n stacked n deep on its Goal
	struct Sample { double t, x, y; int where; };
	std::vector<std::vector<Sample>> tracks;
	std::vector<std::vector<std::pair<double, int>>> faces;
	std::vector<std::array<double, 3>> scores;

	int add(Piece p) {
		pieces.push_back(p);
		return pieces.size() - 1;
	}

	void addGoal(tide::Point p, GoalKind kind, int toggle) { goals.push_back({p.x, p.y, kind, toggle, {}}); }

	void stackOn(Goal& g, int i) {
		Piece& p = pieces[i];
		p.goal = &g - goals.data();
		p.held = false;
		p.x = g.x;
		p.y = g.y;
		g.stack.push_back(i);
	}

	tide::Point tool(const World& w) const { return onRobot(w, robot.claw.right, robot.claw.forward); }

	void moveHeld(const World& w) {
		const tide::Point at = tool(w);
		for (int i : held) { pieces[i].x = at.x; pieces[i].y = at.y; }
	}

	// One Pin and one Cup at most (SG6), so a Cup with its Pin comes up together
	void claw(World& w) {
		const tide::Point at = tool(w);
		if (held.empty() && robot.claw.grab() && robot.claw.lowered()) {
			bool pin = false, cup = false;
			for (std::size_t i = 0; i < pieces.size(); i++) {
				Piece& p = pieces[i];
				if (!p.free() || p.lying || std::hypot(p.x - at.x, p.y - at.y) > robot.claw.reach) continue;
				if (p.cup ? cup : pin) continue;
				(p.cup ? cup : pin) = true;
				p.held = true;
				held.push_back(i);
			}
			if (!held.empty()) w.event("claw picked up %s", pin && cup ? "Pin in Cup" : pin ? "Pin" : "Cup");
		} else if (!held.empty() && robot.claw.release()) {
			Goal* onto = nullptr;
			for (Goal& g : goals)
				if (std::hypot(g.x - at.x, g.y - at.y) < GOAL_R) onto = &g;
			// Pin first, so a Cup carried with it sits on top
			std::stable_sort(held.begin(), held.end(), [&](int a, int b) { return !pieces[a].cup && pieces[b].cup; });
			for (int i : held) {
				if (onto) stackOn(*onto, i);
				else pieces[i].held = false;
			}
			held.clear();
			if (onto) w.event("claw stacked on goal %d", int(onto - goals.data()));
			else w.event("claw dropped on the tiles");
		}
	}

	// Distance from the spinner's contact point to a Toggle's face, or infinity off its length
	double gap(const World& w, const Toggle& t) const {
		const tide::Point c = onRobot(w, robot.spinner.right, robot.spinner.forward);
		const tide::Point n = tide::toField(0, 1, t.inward);
		const double out = (c.x - t.x) * n.x + (c.y - t.y) * n.y;
		const double along = (c.x - t.x) * n.y - (c.y - t.y) * n.x;
		return std::fabs(along) <= 13 ? out : INFINITY;
	}

	// Index of the Toggle the spinner is pressed against, or -1
	int touching(const World& w) const {
		for (std::size_t i = 0; i < toggles.size(); i++)
			if (gap(w, toggles[i]) < 2.5) return i;
		return -1;
	}

	void spin(World& w, double dt) {
		if (!robot.spinner.motor) return;
		const int i = touching(w);
		if (i < 0) return;
		Toggle* t = &toggles[i];
		t->turn += w.motor(robot.spinner.motor).mv / 12000.0 * robot.spinner.facesPerSecond * dt;
		if (std::fabs(t->turn) >= 1) {
			t->face = (t->face + (t->turn > 0 ? 1 : 2)) % 3;
			t->turn = 0;
		}
	}

	// Shoves loose pieces out of the robot's footprint and off the walls; the claw's channel
	// (from the tool point out to the nearest edge) is open, so the robot can back onto a piece
	void push(const World& w) {
		const double h = toRad(w.theta);
		const double c = std::cos(h), s = std::sin(h);
		const auto& k = robot.claw;
		for (Piece& p : pieces) {
			if (!p.free()) continue;
			const double dx = p.x - w.x, dy = p.y - w.y;
			const double right = dx * c - dy * s, forward = dx * s + dy * c;
			const double r = p.radius();
			const bool inChannel = std::fabs(right - k.right) < k.reach &&
				(k.forward < 0 ? forward < k.forward + k.reach : forward > k.forward - k.reach);
			if (inChannel) continue;

			const double pushes[] = {w.width / 2 + r - right, right + w.width / 2 + r, w.front + r - forward, forward + w.back + r};
			if (*std::min_element(pushes, pushes + 4) <= 0) continue;
			const int out = std::min_element(pushes, pushes + 4) - pushes;
			const double nr = out == 0 ? w.width / 2 + r : out == 1 ? -w.width / 2 - r : right;
			const double nf = out == 2 ? w.front + r : out == 3 ? -w.back - r : forward;
			const tide::Point moved = tide::toField(nr, nf, w.theta);
			p.x = w.x + moved.x;
			p.y = w.y + moved.y;
		}
		for (Piece& p : pieces) {
			p.x = std::clamp(p.x, -w.halfField + p.radius(), w.halfField - p.radius());
			p.y = std::clamp(p.y, -w.halfField + p.radius(), w.halfField - p.radius());
		}
	}

	void record(const World& w) {
		if (tracks.size() != pieces.size()) restartLog();
		const double t = (w.sched.now - w.logStart) / 1000.0;
		for (std::size_t i = 0; i < pieces.size(); i++) {
			const Piece& p = pieces[i];
			int where = p.held ? 1 : 0;
			if (p.goal >= 0) {
				const auto& stack = goals[p.goal].stack;
				where = 2 + int(std::find(stack.begin(), stack.end(), int(i)) - stack.begin());
			}
			auto& track = tracks[i];
			if (track.empty() || track.back().where != where || std::hypot(track.back().x - p.x, track.back().y - p.y) > 0.05)
				track.push_back({t, p.x, p.y, where});
		}
		for (std::size_t i = 0; i < toggles.size(); i++)
			if (faces[i].empty() || faces[i].back().second != toggles[i].face) faces[i].push_back({t, toggles[i].face});
		const auto [red, blue] = score(w);
		if (scores.empty() || scores.back()[1] != red || scores.back()[2] != blue) scores.push_back({t, double(red), double(blue)});
	}
};

inline Field field;

// Wires the field into the world; call after the robot's setup so the preload is known
inline void start(World& w) {
	field.setup();
	w.game = [&w] { field.step(w, 0.001); };
	w.optical = [&w](int port) { return field.optical(w, port); };
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace sim::game
