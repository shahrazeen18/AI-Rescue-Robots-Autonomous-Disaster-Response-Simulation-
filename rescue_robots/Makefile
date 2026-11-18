CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
SANFLAGS := -std=c++17 -g -O1 -Wall -Wextra -Wpedantic -fsanitize=address,undefined -fno-sanitize-recover=all
SRC_DIR  := src
BUILD    := build

LIB_SRCS := $(SRC_DIR)/types.cpp $(SRC_DIR)/environment.cpp $(SRC_DIR)/robot.cpp \
            $(SRC_DIR)/rescue_request.cpp $(SRC_DIR)/logger.cpp \
            $(SRC_DIR)/scenario.cpp $(SRC_DIR)/simulation.cpp
LIB_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))
HEADERS  := $(wildcard $(SRC_DIR)/*.h)

all: rescue_sim gen_scenario test_runner fuzz_loader

$(BUILD)/%.o: $(SRC_DIR)/%.cpp $(HEADERS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

rescue_sim: $(LIB_OBJS) $(BUILD)/main.o
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/main.o: $(SRC_DIR)/main.cpp $(HEADERS)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

gen_scenario: tools/gen_scenario.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) $< -o $@

test_runner: tests/test_main.cpp $(LIB_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) tests/test_main.cpp $(LIB_OBJS) -o $@

fuzz_loader: tests/fuzz_loader.cpp $(LIB_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) tests/fuzz_loader.cpp $(LIB_OBJS) -o $@

# Sanitizer builds (AddressSanitizer + UndefinedBehaviorSanitizer)
asan: $(BUILD)/rescue_sim_asan $(BUILD)/test_runner_asan $(BUILD)/fuzz_loader_asan

$(BUILD)/rescue_sim_asan: $(LIB_SRCS) $(SRC_DIR)/main.cpp $(HEADERS)
	@mkdir -p $(BUILD)
	$(CXX) $(SANFLAGS) $(LIB_SRCS) $(SRC_DIR)/main.cpp -o $@

$(BUILD)/test_runner_asan: $(LIB_SRCS) tests/test_main.cpp $(HEADERS)
	@mkdir -p $(BUILD)
	$(CXX) $(SANFLAGS) -I$(SRC_DIR) $(LIB_SRCS) tests/test_main.cpp -o $@

$(BUILD)/fuzz_loader_asan: $(LIB_SRCS) tests/fuzz_loader.cpp $(HEADERS)
	@mkdir -p $(BUILD)
	$(CXX) $(SANFLAGS) -I$(SRC_DIR) $(LIB_SRCS) tests/fuzz_loader.cpp -o $@

test: all asan
	./tests/run_tests.sh

bench: all
	./tools/benchmark.sh

clean:
	rm -rf $(BUILD) rescue_sim gen_scenario test_runner fuzz_loader

.PHONY: all asan test bench clean
