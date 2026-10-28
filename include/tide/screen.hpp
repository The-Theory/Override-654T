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

#include "liblvgl/lvgl.h"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

// Rajdhani SemiBold, generated into src/fonts/ with lv_font_conv (ASCII + °)
extern "C" {
LV_FONT_DECLARE(rajdhani_16)
LV_FONT_DECLARE(rajdhani_18)
LV_FONT_DECLARE(rajdhani_20)
}



namespace tide {

enum class Alliance { RED, BLUE };

// Brain touchscreen: Auton (alliance, options, auton), Tune (tuners + result), Info (live readings)
namespace screen {

////////////////////////////////////////////////////////////////
#pragma region Entries /////////////////////////////////////////
////////////////////////////////////////////////////////////////

struct Auton {
	const char* name;
	void (*run)(Alliance);
};

// Returns text to show when it finishes, e.g. tuned gains
struct Tuner {
	const char* name;
	std::string (*run)();
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region State ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

namespace color {
inline const lv_color_t BG       = lv_color_hex(0x0A1A2F);
inline const lv_color_t BUTTON   = lv_color_hex(0x1C3350);
inline const lv_color_t PRESSED  = lv_color_hex(0x2A4A70);	// Lighter flash while held
inline const lv_color_t PANEL    = lv_color_hex(0x12263F);
inline const lv_color_t DIVIDER  = lv_color_hex(0x4A6A90);
inline const lv_color_t SELECTED = lv_color_hex(0x7FC8F8);	// Light blue
inline const lv_color_t TEXT     = lv_color_hex(0xFFFFFF);
inline const lv_color_t DIM      = lv_color_hex(0x8FA6C2);	// Unselected tabs, info labels
inline const lv_color_t RED      = lv_color_hex(0xC62828);
inline const lv_color_t BLUE     = lv_color_hex(0x1E6FD9);
}

// 468 x 180 page area, split into 56 px rows with 6 px gaps
constexpr int GAP    = 6;
constexpr int ROW    = 56;
constexpr int PAGE_W = 480 - 2 * GAP;
constexpr int PAGE_H = 3 * ROW + 2 * GAP;

// Info page: a key column, then fixed value columns
constexpr int INFO_KEY_W = 86;
constexpr int INFO_COL_W = 120;
constexpr int INFO_COLS  = 3;

inline std::vector<Auton> autons;
inline std::vector<Tuner> tuners;
inline std::vector<const char*> options;
inline std::function<std::string()> info;

// Written by touch callbacks (LVGL task), read by auton/driver tasks; plain ints are fine
inline int autonIdx = 0;
inline int tunerIdx = 0;
inline int pageIdx  = 0;
inline Alliance side = Alliance::RED;
inline std::vector<char> optionOn;

// Tuners run outside the LVGL task, so they hand text over through a buffer the refresh timer picks up
inline char report[512] = "";
inline std::atomic<int> reportVersion = 0;

inline std::vector<lv_obj_t*> autonBtns, tunerBtns, pageBtns, pages;
inline lv_obj_t *sideBtn, *reportLabel;
inline lv_obj_t* infoCols[1 + INFO_COLS];	// keys, then values

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Widgets /////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

// No theme styles, so no gradients, shadows, radius, or transitions
inline lv_obj_t* plain(lv_obj_t* parent) {
	lv_obj_t* obj = lv_obj_create(parent);
	lv_obj_remove_style_all(obj);
	lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
	return obj;
}

inline lv_obj_t* text(lv_obj_t* parent, const char* str, lv_color_t col = color::TEXT) {
	lv_obj_t* obj = lv_label_create(parent);
	lv_label_set_text(obj, str);
	lv_obj_set_style_text_font(obj, &rajdhani_16, 0);
	lv_obj_set_style_text_color(obj, col, 0);
	lv_obj_set_style_text_line_space(obj, 6, 0);
	return obj;
}

// Flat navy; filled with `on` while selected (dark text on light blue)
inline lv_obj_t* button(lv_obj_t* parent, const char* name, int w, lv_color_t on, lv_event_cb_t onClick, int index) {
	lv_obj_t* btn = plain(parent);
	lv_obj_set_size(btn, w, ROW);
	lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
	lv_obj_set_style_bg_color(btn, color::BUTTON, 0);
	lv_obj_set_style_bg_color(btn, color::PRESSED, LV_STATE_PRESSED);
	lv_obj_set_style_bg_color(btn, on, LV_STATE_CHECKED);
	lv_obj_set_style_text_color(btn, color::TEXT, 0);
	if (lv_color_eq(on, color::SELECTED)) lv_obj_set_style_text_color(btn, color::BG, LV_STATE_CHECKED);
	lv_obj_t* label = lv_label_create(btn);	// No color of its own, so it follows the button's checked state
	lv_label_set_text(label, name);
	lv_obj_set_style_text_font(label, &rajdhani_20, 0);
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, onClick, LV_EVENT_CLICKED, (void*)(intptr_t)index);
	return btn;
}

inline int clickedIndex(lv_event_t* e) { return (int)(intptr_t)lv_event_get_user_data(e); }

inline void check(const std::vector<lv_obj_t*>& btns, int selected) {
	for (int i = 0; i < (int)btns.size(); i++) lv_obj_set_state(btns[i], LV_STATE_CHECKED, i == selected);
}

// Stacks children top to bottom with the standard gap
inline lv_obj_t* column(lv_obj_t* parent, int x, int w) {
	lv_obj_t* obj = plain(parent);
	lv_obj_set_pos(obj, x, 0);
	lv_obj_set_size(obj, w, PAGE_H);
	lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_style_pad_gap(obj, GAP, 0);
	return obj;
}

// Slightly lighter block behind text output
inline lv_obj_t* panel(lv_obj_t* parent, int x, int w) {
	lv_obj_t* obj = plain(parent);
	lv_obj_set_pos(obj, x, 0);
	lv_obj_set_size(obj, w, PAGE_H);
	lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
	lv_obj_set_style_bg_color(obj, color::PANEL, 0);
	lv_obj_set_style_pad_hor(obj, 12, 0);
	lv_obj_set_style_pad_ver(obj, 10, 0);
	return obj;
}

// Area under the tabs
inline lv_obj_t* page(lv_obj_t* scr) {
	lv_obj_t* obj = plain(scr);
	lv_obj_set_pos(obj, GAP, 40 + 2 * GAP);
	lv_obj_set_size(obj, PAGE_W, PAGE_H);
	pages.push_back(obj);
	return obj;
}

inline void showPage(int idx) {
	pageIdx = idx;
	for (int i = 0; i < (int)pages.size(); i++) {
		if (i == idx) lv_obj_remove_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
		else          lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
	}
	check(pageBtns, idx);
}

// The alliance button is always filled, so its color and name are the selection
inline void showSide() {
	const bool red = side == Alliance::RED;
	lv_obj_set_style_bg_color(sideBtn, red ? color::RED : color::BLUE, LV_STATE_CHECKED);
	lv_label_set_text(lv_obj_get_child(sideBtn, 0), red ? "Red" : "Blue");
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Pages ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

// Alliance + options on the left, divider, autons in a grid that goes 3 wide past 6
inline void buildAutonPage(lv_obj_t* scr) {
	lv_obj_t* p = page(scr);
	constexpr int LEFT_W = 96;

	lv_obj_t* left = column(p, 0, LEFT_W);
	sideBtn = button(left, "", LEFT_W, color::RED, [](lv_event_t*) {
		side = side == Alliance::RED ? Alliance::BLUE : Alliance::RED;
		showSide();
	}, 0);
	lv_obj_add_state(sideBtn, LV_STATE_CHECKED);
	for (int i = 0; i < (int)options.size(); i++) {
		button(left, options[i], LEFT_W, color::SELECTED, [](lv_event_t* e) {
			const int idx = clickedIndex(e);
			optionOn[idx] = !optionOn[idx];
			lv_obj_set_state(lv_event_get_target_obj(e), LV_STATE_CHECKED, optionOn[idx]);
		}, i);
	}

	lv_obj_t* divider = plain(p);
	lv_obj_set_pos(divider, LEFT_W + GAP, 0);
	lv_obj_set_size(divider, 1, PAGE_H);
	lv_obj_set_style_bg_opa(divider, LV_OPA_COVER, 0);
	lv_obj_set_style_bg_color(divider, color::DIVIDER, 0);

	const int gridX = LEFT_W + 2 * GAP + 2;
	const int gridW = PAGE_W - gridX;
	const int cols  = autons.size() > 6 ? 3 : 2;
	lv_obj_t* grid = plain(p);
	lv_obj_set_pos(grid, gridX, 0);
	lv_obj_set_size(grid, gridW, PAGE_H);
	lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
	lv_obj_set_style_pad_gap(grid, GAP, 0);
	for (int i = 0; i < (int)autons.size(); i++) {
		autonBtns.push_back(button(grid, autons[i].name, (gridW - GAP * (cols - 1)) / cols, color::SELECTED, [](lv_event_t* e) {
			autonIdx = clickedIndex(e);
			check(autonBtns, autonIdx);
		}, i));
	}
}

inline void buildTunePage(lv_obj_t* scr) {
	lv_obj_t* p = page(scr);
	constexpr int LEFT_W = 152;
	lv_obj_t* left = column(p, 0, LEFT_W);
	for (int i = 0; i < (int)tuners.size(); i++) {
		tunerBtns.push_back(button(left, tuners[i].name, LEFT_W, color::SELECTED, [](lv_event_t* e) {
			tunerIdx = clickedIndex(e);
			check(tunerBtns, tunerIdx);
		}, i));
	}
	reportLabel = text(panel(p, LEFT_W + GAP, PAGE_W - LEFT_W - GAP), "");
	lv_obj_set_width(reportLabel, LV_PCT(100));
}

inline void buildInfoPage(lv_obj_t* scr) {
	lv_obj_t* box = panel(page(scr), 0, PAGE_W);
	for (int i = 0; i <= INFO_COLS; i++) {
		infoCols[i] = text(box, "", i == 0 ? color::DIM : color::TEXT);
		lv_obj_set_x(infoCols[i], i == 0 ? 0 : INFO_KEY_W + (i - 1) * INFO_COL_W);
	}
}

// Info lines are "Key\tvalue\tvalue"; each tab starts a fixed column, since digits vary in width
inline void showInfo(const std::string& str) {
	std::string cols[1 + INFO_COLS];
	size_t start = 0;
	while (start < str.size()) {
		size_t end = str.find('\n', start);
		if (end == std::string::npos) end = str.size();
		int col = 0;
		for (size_t i = start; i < end; i++) {
			if (str[i] != '\t') cols[col] += str[i];
			else if (col < INFO_COLS) col++;
			else cols[col] += ' ';
		}
		for (std::string& c : cols) c += '\n';
		start = end + 1;
	}
	for (int i = 0; i <= INFO_COLS; i++) lv_label_set_text(infoCols[i], cols[i].c_str());
}

// Runs in the LVGL task, so it's the only place outside touch callbacks that touches widgets
inline void refresh(lv_timer_t*) {
	static int seenReport = -1;
	if (seenReport != reportVersion) {
		seenReport = reportVersion;
		lv_label_set_text(reportLabel, report);
	}
	if (info && pageIdx == 2) showInfo(info());
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region API /////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Builds the screen. Call once in initialize().
 * - info: text for the Info page as "Key\tvalues" lines, refreshed 10x a second while it's open
 * - options: on/off toggles under the alliance button (room for 2), read with option()
 */
inline void init(std::vector<Auton> autons, std::vector<Tuner> tuners, std::function<std::string()> info = {},
				 std::vector<const char*> options = {}) {
	using namespace detail;
	detail::autons  = autons;
	detail::tuners  = tuners;
	detail::info    = info;
	detail::options = options;
	optionOn.assign(options.size(), false);

	lv_obj_t* scr = lv_screen_active();
	lv_obj_remove_style_all(scr);
	lv_obj_set_style_bg_color(scr, color::BG, 0);
	lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
	lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

	// Tabs along the top: text only until selected
	lv_obj_t* nav = plain(scr);
	lv_obj_set_pos(nav, GAP, GAP);
	lv_obj_set_size(nav, PAGE_W, 40);
	lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
	lv_obj_set_style_pad_gap(nav, GAP, 0);
	const char* names[] = {"Auton", "Tune", "Info"};
	for (int i = 0; i < 3; i++) {
		lv_obj_t* tab = button(nav, names[i], (PAGE_W - 2 * GAP) / 3, color::SELECTED, [](lv_event_t* e) { showPage(clickedIndex(e)); }, i);
		lv_obj_set_height(tab, 40);
		lv_obj_set_style_bg_opa(tab, LV_OPA_TRANSP, 0);
		lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, LV_STATE_PRESSED);
		lv_obj_set_style_bg_opa(tab, LV_OPA_COVER, LV_STATE_CHECKED);
		lv_obj_set_style_text_color(tab, color::DIM, 0);
		lv_obj_set_style_text_font(lv_obj_get_child(tab, 0), &rajdhani_18, 0);
		pageBtns.push_back(tab);
	}

	buildAutonPage(scr);
	buildTunePage(scr);
	buildInfoPage(scr);

	showSide();
	check(autonBtns, autonIdx);
	check(tunerBtns, tunerIdx);
	showPage(0);
	lv_timer_create(refresh, 100, nullptr);
}

inline Alliance alliance() { return detail::side; }

// Whether the named option toggle is on; false for names not passed to init()
inline bool option(const char* name) {
	for (int i = 0; i < (int)detail::options.size(); i++)
		if (std::strcmp(detail::options[i], name) == 0) return detail::optionOn[i];
	return false;
}

// Plays the selected auton for the selected alliance
inline void runAuton() {
	if (!detail::autons.empty()) detail::autons[detail::autonIdx].run(detail::side);
}

// Runs the selected tuner and shows what it returns on the Tune page
inline void runTuner() {
	if (detail::tuners.empty()) return;
	const std::string result = detail::tuners[detail::tunerIdx].run();
	std::snprintf(detail::report, sizeof(detail::report), "%s", result.c_str());
	detail::reportVersion++;
}

// For info text: a distance reading in inches, or "--" with nothing in range
inline std::string inches(pros::Distance& sensor) {
	const std::int32_t mm = sensor.get();
	if (mm == PROS_ERR || mm >= 9999) return "--";
	char str[16];
	std::snprintf(str, sizeof(str), "%.1f", mm / 25.4);
	return str;
}

// For info text: the hottest motor in a group [°C]
inline double hottest(pros::MotorGroup& motors) {
	const std::vector<double> temps = motors.get_temperature_all();
	return temps.empty() ? 0 : *std::max_element(temps.begin(), temps.end());
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace screen
} // namespace tide
