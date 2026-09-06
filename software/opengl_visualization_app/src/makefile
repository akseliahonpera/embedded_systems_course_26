CXX := g++

CXXFLAGS_RELEASE := -Wall -std=c++20 -O2 -march=native
CXXFLAGS_DEBUG   := -Wall -std=c++20 -g -Og

LDFLAGS_RELEASE  := -s
LDFLAGS_DEBUG    :=

BINARY_NAME := visualization_app


LIBS := glfw GL GLEW
SRCS := $(wildcard *.cpp) $(wildcard imgui/*.cpp) $(wildcard lodepng/*.cpp)
OBJS_RELEASE := $(patsubst %.cpp, build/release/obj/%.o, $(SRCS))
OBJS_DEBUG := $(patsubst %.cpp, build/debug/obj/%.o, $(SRCS))
DEPS := $(OBJS_RELEASE:.o=.d) $(OBJS_DEBUG:.o=.d)

DEPFLAGS := -MMD -MP

TARGET_RELEASE := build/release/bin/$(BINARY_NAME)
TARGET_DEBUG := build/debug/bin/$(BINARY_NAME)

.PHONY: all debug clean

all: $(TARGET_RELEASE)
debug: $(TARGET_DEBUG)


$(TARGET_RELEASE): $(OBJS_RELEASE)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS_RELEASE) $(LDFLAGS_RELEASE) -o $@ $^ $(addprefix -l,$(LIBS))

$(TARGET_DEBUG): $(OBJS_DEBUG)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS_DEBUG) $(LDFLAGS_DEBUG) -o $@ $^ $(addprefix -l,$(LIBS))

build/release/obj/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS_RELEASE) $(DEPFLAGS) -c $< -o $@

build/debug/obj/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS_DEBUG) $(DEPFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	rm -rf build
