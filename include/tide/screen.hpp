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
#include "pros/misc.hpp"
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>



namespace tide {

enum class Alliance { RED, BLUE };

// Brain touchscreen: pick an auton and alliance, pick and run tuners, watch live robot info
namespace screen {

////////////////////////////////////////////////////////////////
#pragma region Entries /////////////////////////////////////////
////////////////////////////////////////////////////////////////

struct Auton {
	const char* name;
	void (*run)(Alliance);
	const char* note = "";	// One line shown under the list
};

// Returns what to show when it finishes, e.g. tuned gains
struct Tuner {
	const char* name;
	std::string (*run)();
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Theme ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace theme {
inline const lv_color_t BG     = lv_color_hex(0x0B1622);	// Deep water
inline const lv_color_t PANEL  = lv_color_hex(0x15293D);
inline const lv_color_t ACCENT = lv_color_hex(0x1EC8E6);	// Surf
inline const lv_color_t TEXT   = lv_color_hex(0xE6F1F8);
inline const lv_color_t MUTED  = lv_color_hex(0x7C93A8);
inline const lv_color_t RED    = lv_color_hex(0xE5484D);
inline const lv_color_t BLUE   = lv_color_hex(0x3B82F6);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region State ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {
inline std::vector<Auton> autons;
inline std::vector<Tuner> tuners;
inline std::function<std::string()> info;

// Written by touch callbacks (LVGL task), read by auton/driver tasks; plain ints are fine
inline int autonIdx = 0;
inline int tunerIdx = 0;
inline Alliance side = Alliance::RED;

// Tuners run outside the LVGL task, so they hand text over through a buffer the refresh timer picks up
inline char report[512] = "Pick a tuner, then press RIGHT on the controller.";
inline std::atomic<int> reportVersion = 1;

inline std::vector<lv_obj_t*> autonBtns, tunerBtns;
inline lv_obj_t *redBtn, *blueBtn, *selectedLabel, *batteryLabel, *noteLabel, *reportLabel, *infoLabel;

inline void setReport(const std::string& text) {
	std::snprintf(report, sizeof(report), "%s", text.c_str());
	reportVersion++;
}
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Widgets /////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

inline lv_obj_t* panel(lv_obj_t* parent) {
	lv_obj_t* obj = lv_obj_create(parent);
	lv_obj_set_style_bg_color(obj, theme::PANEL, 0);
	lv_obj_set_style_border_width(obj, 0, 0);
	lv_obj_set_style_radius(obj, 8, 0);
	lv_obj_set_style_pad_all(obj, 6, 0);
	return obj;
}

inline lv_obj_t* label(lv_obj_t* parent, const char* text, const lv_font_t* font, lv_color_t color) {
	lv_obj_t* obj = lv_label_create(parent);
	lv_label_set_text(obj, text);
	lv_obj_set_style_text_font(obj, font, 0);
	lv_obj_set_style_text_color(obj, color, 0);
	return obj;
}

// Selected state (LV_STATE_CHECKED) fills with `on` and darkens the text
inline lv_obj_t* button(lv_obj_t* parent, const char* text, lv_color_t on, lv_event_cb_t onClick, int index) {
	lv_obj_t* btn = lv_button_create(parent);
	lv_obj_set_style_bg_color(btn, theme::BG, 0);
	lv_obj_set_style_bg_color(btn, on, LV_STATE_CHECKED);
	lv_obj_set_style_border_color(btn, on, 0);
	lv_obj_set_style_border_width(btn, 2, 0);
	lv_obj_set_style_radius(btn, 8, 0);
	lv_obj_set_style_shadow_width(btn, 0, 0);
	lv_obj_t* text_ = label(btn, text, &lv_font_montserrat_16, theme::TEXT);
	lv_obj_set_style_text_color(text_, theme::BG, LV_STATE_CHECKED);
	lv_obj_center(text_);
	lv_obj_add_event_cb(btn, onClick, LV_EVENT_CLICKED, (void*)(intptr_t)index);
	return btn;
}

inline void check(const std::vector<lv_obj_t*>& btns, int selected) {
	for (int i = 0; i < (int)btns.size(); i++) {
		if (i == selected) lv_obj_add_state(btns[i], LV_STATE_CHECKED);
		else               lv_obj_remove_state(btns[i], LV_STATE_CHECKED);
	}
}

inline void refreshSelection() {
	check(autonBtns, autonIdx);
	check(tunerBtns, tunerIdx);
	check({redBtn, blueBtn}, side == Alliance::RED ? 0 : 1);
	if (autons.empty()) return;
	lv_label_set_text_fmt(selectedLabel, "%s  " LV_SYMBOL_RIGHT "  %s", autons[autonIdx].name, side == Alliance::RED ? "RED" : "BLUE");
	lv_obj_set_style_text_color(selectedLabel, side == Alliance::RED ? theme::RED : theme::BLUE, 0);
	lv_label_set_text(noteLabel, autons[autonIdx].note);
}

inline int clickedIndex(lv_event_t* e) { return (int)(intptr_t)lv_event_get_user_data(e); }

// Fills the flex grid of a list tab
inline lv_obj_t* grid(lv_obj_t* parent, int width) {
	lv_obj_t* obj = lv_obj_create(parent);
	lv_obj_set_size(obj, width, LV_PCT(100));
	lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(obj, 0, 0);
	lv_obj_set_style_pad_all(obj, 0, 0);
	lv_obj_set_style_pad_gap(obj, 6, 0);
	lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW_WRAP);
	return obj;
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Layout //////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

inline void buildHeader(lv_obj_t* scr, const char* title) {
	lv_obj_t* bar = panel(scr);
	lv_obj_set_size(bar, 480, 34);
	lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
	lv_obj_set_style_radius(bar, 0, 0);
	lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_align(label(bar, title, &lv_font_montserrat_18, theme::ACCENT), LV_ALIGN_LEFT_MID, 4, 0);
	selectedLabel = label(bar, "", &lv_font_montserrat_16, theme::TEXT);
	lv_obj_align(selectedLabel, LV_ALIGN_CENTER, 20, 0);
	batteryLabel = label(bar, "", &lv_font_montserrat_14, theme::MUTED);
	lv_obj_align(batteryLabel, LV_ALIGN_RIGHT_MID, -4, 0);
}

inline void buildAutonTab(lv_obj_t* tab) {
	// Alliance column
	lv_obj_t* sideCol = lv_obj_create(tab);
	lv_obj_set_size(sideCol, 96, LV_PCT(100));
	lv_obj_set_style_bg_opa(sideCol, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(sideCol, 0, 0);
	lv_obj_set_style_pad_all(sideCol, 0, 0);
	lv_obj_set_style_pad_gap(sideCol, 6, 0);
	lv_obj_set_flex_flow(sideCol, LV_FLEX_FLOW_COLUMN);
	lv_obj_remove_flag(sideCol, LV_OBJ_FLAG_SCROLLABLE);
	auto onSide = [](lv_event_t* e) {
		side = clickedIndex(e) == 0 ? Alliance::RED : Alliance::BLUE;
		refreshSelection();
	};
	redBtn  = button(sideCol, "RED",  theme::RED,  onSide, 0);
	blueBtn = button(sideCol, "BLUE", theme::BLUE, onSide, 1);
	for (lv_obj_t* btn : {redBtn, blueBtn}) lv_obj_set_size(btn, LV_PCT(100), 64);

	// Auton grid + note
	lv_obj_t* right = grid(tab, 356);
	lv_obj_align(right, LV_ALIGN_TOP_RIGHT, 0, 0);
	auto onAuton = [](lv_event_t* e) {
		autonIdx = clickedIndex(e);
		refreshSelection();
	};
	for (int i = 0; i < (int)autons.size(); i++) {
		autonBtns.push_back(button(right, autons[i].name, theme::ACCENT, onAuton, i));
		lv_obj_set_size(autonBtns.back(), 172, 44);
	}
	noteLabel = label(right, "", &lv_font_montserrat_14, theme::MUTED);
	lv_obj_set_width(noteLabel, LV_PCT(100));
	lv_label_set_long_mode(noteLabel, LV_LABEL_LONG_WRAP);
}

inline void buildTuneTab(lv_obj_t* tab) {
	lv_obj_t* left = grid(tab, 160);
	auto onTuner = [](lv_event_t* e) {
		tunerIdx = clickedIndex(e);
		refreshSelection();
	};
	for (int i = 0; i < (int)tuners.size(); i++) {
		tunerBtns.push_back(button(left, tuners[i].name, theme::ACCENT, onTuner, i));
		lv_obj_set_size(tunerBtns.back(), LV_PCT(100), 36);
	}

	lv_obj_t* box = panel(tab);
	lv_obj_set_size(box, 288, LV_PCT(100));
	lv_obj_align(box, LV_ALIGN_TOP_RIGHT, 0, 0);
	reportLabel = label(box, report, &lv_font_montserrat_14, theme::TEXT);
	lv_obj_set_width(reportLabel, LV_PCT(100));
	lv_label_set_long_mode(reportLabel, LV_LABEL_LONG_WRAP);
}

inline void buildInfoTab(lv_obj_t* tab) {
	infoLabel = label(tab, "", &lv_font_montserrat_14, theme::TEXT);
	lv_obj_set_width(infoLabel, LV_PCT(100));
}

// Runs in the LVGL task, so it's the only place outside touch callbacks that touches widgets
inline void refresh(lv_timer_t*) {
	static int seenReport = 0;
	if (seenReport != reportVersion) {
		seenReport = reportVersion;
		lv_label_set_text(reportLabel, report);
	}
	if (info) lv_label_set_text(infoLabel, info().c_str());
	lv_label_set_text_fmt(batteryLabel, LV_SYMBOL_BATTERY_FULL " %d%%", (int)pros::battery::get_capacity());
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region API /////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Builds the screen. Call once in initialize().
 * - info: text for the INFO tab, refreshed 10x a second
 */
inline void init(std::vector<Auton> autons, std::vector<Tuner> tuners,
				 std::function<std::string()> info = {}, const char* title = "TIDE") {
	using namespace detail;
	detail::autons = autons;
	detail::tuners = tuners;
	detail::info   = info;

	lv_obj_t* scr = lv_screen_active();
	lv_obj_set_style_bg_color(scr, theme::BG, 0);
	lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
	buildHeader(scr, title);

	lv_obj_t* tabs = lv_tabview_create(scr);
	lv_obj_set_size(tabs, 480, 206);
	lv_obj_align(tabs, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_set_style_bg_color(tabs, theme::BG, 0);
	lv_tabview_set_tab_bar_size(tabs, 30);
	lv_obj_remove_flag(lv_tabview_get_content(tabs), LV_OBJ_FLAG_SCROLLABLE);	// No accidental swipes between tabs

	lv_obj_t* tabBar = lv_tabview_get_tab_bar(tabs);
	lv_obj_set_style_bg_color(tabBar, theme::BG, 0);
	auto addTab = [&](const char* name, void (*build)(lv_obj_t*)) {
		lv_obj_t* tab = lv_tabview_add_tab(tabs, name);
		lv_obj_set_style_pad_all(tab, 6, 0);
		build(tab);
	};
	addTab("AUTON", buildAutonTab);
	addTab("TUNE",  buildTuneTab);
	addTab("INFO",  buildInfoTab);
	for (uint32_t i = 0; i < lv_obj_get_child_count(tabBar); i++) {
		lv_obj_t* tabBtn = lv_obj_get_child(tabBar, i);
		lv_obj_set_style_bg_opa(tabBtn, LV_OPA_TRANSP, 0);
		lv_obj_set_style_text_color(tabBtn, theme::MUTED, 0);
		lv_obj_set_style_text_color(tabBtn, theme::ACCENT, LV_STATE_CHECKED);
		lv_obj_set_style_border_color(tabBtn, theme::ACCENT, LV_STATE_CHECKED);
		lv_obj_set_style_text_font(tabBtn, &lv_font_montserrat_16, 0);
	}

	refreshSelection();
	lv_timer_create(refresh, 100, nullptr);
}

inline Alliance alliance() { return detail::side; }

// Plays the selected auton for the selected alliance
inline void runAuton() {
	if (!detail::autons.empty()) detail::autons[detail::autonIdx].run(detail::side);
}

// Runs the selected tuner and shows what it returns on the TUNE tab
inline void runTuner() {
	if (detail::tuners.empty()) return;
	const Tuner& tuner = detail::tuners[detail::tunerIdx];
	detail::setReport(std::string("Running ") + tuner.name + "...");
	detail::setReport(tuner.run());
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace screen
} // namespace tide
