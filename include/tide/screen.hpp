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
#include <functional>
#include <string>
#include <vector>



namespace tide {

enum class Alliance { RED, BLUE };

// Brain touchscreen: Auton (alliance + auton), Tune (tuners + result), Info (live readings)
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
inline const lv_color_t BG       = lv_color_hex(0x000000);
inline const lv_color_t BUTTON   = lv_color_hex(0x2A2A2A);
inline const lv_color_t PRESSED  = lv_color_hex(0x505050);
inline const lv_color_t SELECTED = lv_color_hex(0xFFFFFF);
inline const lv_color_t TEXT     = lv_color_hex(0xFFFFFF);
inline const lv_color_t RED      = lv_color_hex(0xC62828);
inline const lv_color_t BLUE     = lv_color_hex(0x1565C0);
}

inline std::vector<Auton> autons;
inline std::vector<Tuner> tuners;
inline std::function<std::string()> info;

// Written by touch callbacks (LVGL task), read by auton/driver tasks; plain ints are fine
inline int autonIdx = 0;
inline int tunerIdx = 0;
inline int pageIdx  = 0;
inline Alliance side = Alliance::RED;

// Tuners run outside the LVGL task, so they hand text over through a buffer the refresh timer picks up
inline char report[512] = "";
inline std::atomic<int> reportVersion = 0;

inline std::vector<lv_obj_t*> autonBtns, tunerBtns, sideBtns, pageBtns, pages;
inline lv_obj_t *reportLabel, *infoLabel;

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

inline lv_obj_t* text(lv_obj_t* parent, const char* str) {
	lv_obj_t* obj = lv_label_create(parent);
	lv_label_set_text(obj, str);
	lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, 0);
	lv_obj_set_style_text_color(obj, color::TEXT, 0);
	return obj;
}

// Flat gray; filled with `on` while selected
inline lv_obj_t* button(lv_obj_t* parent, const char* name, int w, lv_color_t on, lv_event_cb_t onClick, int index) {
	lv_obj_t* btn = plain(parent);
	lv_obj_set_size(btn, w, 56);
	lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
	lv_obj_set_style_bg_color(btn, color::BUTTON, 0);
	lv_obj_set_style_bg_color(btn, color::PRESSED, LV_STATE_PRESSED);
	lv_obj_set_style_bg_color(btn, on, LV_STATE_CHECKED);
	lv_obj_set_style_text_color(btn, color::TEXT, 0);
	if (lv_color_eq(on, color::SELECTED)) lv_obj_set_style_text_color(btn, color::BG, LV_STATE_CHECKED);
	lv_obj_t* label = lv_label_create(btn);	// No color of its own, so it follows the button's checked state
	lv_label_set_text(label, name);
	lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, onClick, LV_EVENT_CLICKED, (void*)(intptr_t)index);
	return btn;
}

inline int clickedIndex(lv_event_t* e) { return (int)(intptr_t)lv_event_get_user_data(e); }

inline void check(const std::vector<lv_obj_t*>& btns, int selected) {
	for (int i = 0; i < (int)btns.size(); i++) {
		if (i == selected) lv_obj_add_state(btns[i], LV_STATE_CHECKED);
		else               lv_obj_remove_state(btns[i], LV_STATE_CHECKED);
	}
}

// Area under the page buttons; contents wrap left to right with 4 px gaps
inline lv_obj_t* page(lv_obj_t* scr) {
	lv_obj_t* obj = plain(scr);
	lv_obj_set_pos(obj, 0, 60);
	lv_obj_set_size(obj, 480, 180);
	lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW_WRAP);
	lv_obj_set_style_pad_gap(obj, 4, 0);
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

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Pages ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

inline void buildAutonPage(lv_obj_t* scr) {
	lv_obj_t* p = page(scr);
	auto onSide = [](lv_event_t* e) {
		side = clickedIndex(e) == 0 ? Alliance::RED : Alliance::BLUE;
		check(sideBtns, clickedIndex(e));
	};
	sideBtns = {button(p, "Red", 238, color::RED, onSide, 0), button(p, "Blue", 238, color::BLUE, onSide, 1)};
	for (int i = 0; i < (int)autons.size(); i++) {
		autonBtns.push_back(button(p, autons[i].name, 157, color::SELECTED, [](lv_event_t* e) {
			autonIdx = clickedIndex(e);
			check(autonBtns, autonIdx);
		}, i));
	}
}

inline void buildTunePage(lv_obj_t* scr) {
	lv_obj_t* p = page(scr);
	for (int i = 0; i < (int)tuners.size(); i++) {
		tunerBtns.push_back(button(p, tuners[i].name, 117, color::SELECTED, [](lv_event_t* e) {
			tunerIdx = clickedIndex(e);
			check(tunerBtns, tunerIdx);
		}, i));
	}
	reportLabel = text(p, "");
	lv_obj_set_width(reportLabel, 480);
}

inline void buildInfoPage(lv_obj_t* scr) {
	infoLabel = text(page(scr), "");
}

// Runs in the LVGL task, so it's the only place outside touch callbacks that touches widgets
inline void refresh(lv_timer_t*) {
	static int seenReport = -1;
	if (seenReport != reportVersion) {
		seenReport = reportVersion;
		lv_label_set_text(reportLabel, report);
	}
	if (info && pageIdx == 2) lv_label_set_text(infoLabel, info().c_str());
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region API /////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Builds the screen. Call once in initialize().
 * - info: text for the Info page, refreshed 10x a second while it's open
 */
inline void init(std::vector<Auton> autons, std::vector<Tuner> tuners, std::function<std::string()> info = {}) {
	using namespace detail;
	detail::autons = autons;
	detail::tuners = tuners;
	detail::info   = info;

	lv_obj_t* scr = lv_screen_active();
	lv_obj_remove_style_all(scr);
	lv_obj_set_style_bg_color(scr, color::BG, 0);
	lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
	lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

	// Page buttons along the top
	lv_obj_t* nav = plain(scr);
	lv_obj_set_size(nav, 480, 56);
	lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
	lv_obj_set_style_pad_gap(nav, 4, 0);
	const char* names[] = {"Auton", "Tune", "Info"};
	for (int i = 0; i < 3; i++)
		pageBtns.push_back(button(nav, names[i], 157, color::SELECTED, [](lv_event_t* e) { showPage(clickedIndex(e)); }, i));

	buildAutonPage(scr);
	buildTunePage(scr);
	buildInfoPage(scr);

	check(sideBtns, side == Alliance::RED ? 0 : 1);
	check(autonBtns, autonIdx);
	check(tunerBtns, tunerIdx);
	showPage(0);
	lv_timer_create(refresh, 100, nullptr);
}

inline Alliance alliance() { return detail::side; }

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
