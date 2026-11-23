# Desktop sim: the robot's autons and Tide against fake PROS and LemLib, on the host compiler.
# Separate from the brain Makefile; see docs/sim.md

ROOT     := ../..
OUT      := out
INCLUDES := -I fake -I . -I $(ROOT)/include -I $(ROOT)/src
CXXFLAGS := -std=c++20 -O2 -g -Wall $(INCLUDES)
SRC      := $(shell find $(ROOT)/src -name '*.cpp')
HEADERS  := $(shell find fake . $(ROOT)/include/tide $(ROOT)/src -name '*.hpp' -o -name '*.h' -o -name '*.inc')
AUTON    ?=
WEB      ?= web/public/sim.js

# One self-contained ES module (wasm inlined) exporting createSim(); a fresh instance per run,
# since the robot's globals only start clean once. Fibers need Asyncify.
EMFLAGS  := -std=c++20 -O2 $(INCLUDES) -sASYNCIFY -sASYNCIFY_STACK_SIZE=131072 -sSTACK_SIZE=1048576 \
            -sMODULARIZE -sEXPORT_ES6 -sEXPORT_NAME=createSim -sSINGLE_FILE -sENVIRONMENT=web \
            -sINVOKE_RUN=0 -sEXIT_RUNTIME=0 -sALLOW_MEMORY_GROWTH -sEXPORTED_RUNTIME_METHODS=callMain,FS

.PHONY: run check robust web view

run: $(OUT)/sim
	$(OUT)/sim "$(AUTON)" --out $(OUT) --field $(ROOT)/docs/field-map.svg

check: $(OUT)/check
	$(OUT)/check

# AUTON on sloppy setups (right back dheading), off distance sensors and hand-set pieces; every line should score the same
# Forward (-back) only: autons start with the back flush on a wall, so it can't be set into it
SETUPS := "0 0 0" "1 0 0" "-1 0 0" "0 -1 0" "0 -2 0" "0 -1 3" "0 -1 -3" "1 -1 2" "-1 -1 -2" "1 -2 -3"
robust: $(OUT)/sim
	@for s in $(SETUPS); do for k in 1 1.03 0.97; do for j in 0 1.5; do \
		printf "setup %-9s sensors x%-4s pieces ±%-3s " "$$s" $$k $$j; \
		$(OUT)/sim "$(AUTON)" $(ARGS) --setup $$s --distance-scale $$k --jitter $$j --out $(OUT)/robust | \
			awk '/still running/{late=1} /FAILED/{f++} /^Score/{s=$$3} END{printf "score %s%s%s\n", s, late?"  LATE":"", f?"  ("f" failed)":""}'; \
	done; done; done

web: $(WEB)

# Builds the browser sim and serves the viewer
view: web
	cd web && bun install && bun run dev

$(WEB): main.cpp $(SRC) $(HEADERS)
	@mkdir -p $(dir $(WEB))
	em++ $(EMFLAGS) main.cpp $(SRC) -o $@

$(OUT)/sim: main.cpp $(SRC) $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) main.cpp $(SRC) -o $@

$(OUT)/check: check.cpp $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) check.cpp -o $@
