CXX ?= c++
CXXFLAGS ?= -std=c++17 -O3 -Wall -Wextra -Wpedantic
CPPFLAGS += -Isrc -Ibaseline
BUILD := work-build

.PHONY: all test test-tools replay benchmark clean
all: $(BUILD)/navigation_tests $(BUILD)/physics_tests $(BUILD)/trajectory_tests $(BUILD)/benchmark
$(BUILD):
	mkdir -p $@
$(BUILD)/navigation_tests: tests/navigation_tests.cpp src/navigation.cpp src/navigation.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/navigation_tests.cpp src/navigation.cpp -o $@
$(BUILD)/physics_tests: tests/physics_tests.cpp src/navigation.cpp src/navigation.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/physics_tests.cpp src/navigation.cpp -o $@
$(BUILD)/trajectory_tests: tests/trajectory_tests.cpp src/navigation.cpp src/navigation.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tests/trajectory_tests.cpp src/navigation.cpp -o $@
$(BUILD)/benchmark: simulation/benchmark.cpp src/navigation.cpp src/navigation.h baseline/algorithm_core.cpp baseline/data_filtration.cpp baseline/algorithm_core.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) simulation/benchmark.cpp src/navigation.cpp baseline/algorithm_core.cpp baseline/data_filtration.cpp -o $@
$(BUILD)/replay: tools/replay.cpp src/navigation.cpp src/navigation.h | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) tools/replay.cpp src/navigation.cpp -o $@
$(BUILD)/test_adapter: tools/test_adapter.cpp src/navigation.cpp src/navigation.h firmware/imu_lora/acquisition.h firmware/imu_lora/telemetry.h | $(BUILD)
	$(CXX) $(CPPFLAGS) -Ifirmware/imu_lora $(CXXFLAGS) tools/test_adapter.cpp src/navigation.cpp -o $@
$(BUILD)/test_st_driver: tools/test_st_driver.cpp firmware/st_imu_lora/st_driver.h | $(BUILD)
	$(CXX) $(CPPFLAGS) -Ifirmware/st_imu_lora $(CXXFLAGS) tools/test_st_driver.cpp -o $@
replay: $(BUILD)/replay
test-tools: $(BUILD)/test_adapter $(BUILD)/test_st_driver
	python3 tools/test_tools.py
	python3 tools/test_magnetometer_fit.py
	./$(BUILD)/test_adapter
	./$(BUILD)/test_st_driver
test: $(BUILD)/navigation_tests $(BUILD)/physics_tests $(BUILD)/trajectory_tests
	./$(BUILD)/navigation_tests
	./$(BUILD)/physics_tests
	./$(BUILD)/trajectory_tests
benchmark: $(BUILD)/benchmark
	./$(BUILD)/benchmark --hours 8 --dt 0.02 --seeds 3 --output simulation/results
clean:
	rm -rf $(BUILD)
