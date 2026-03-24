CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-unused-result
LDFLAGS = -lglfw -lGLEW -lGL -lcurl

IMGUI_DIR = imgui_lib
IMGUI_SRC = $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp \
            $(IMGUI_DIR)/imgui_widgets.cpp \
            $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

TARGET = drone_overlay
SRC = main.cpp $(IMGUI_SRC)

$(TARGET): $(SRC) stb_image.h
	$(CXX) $(CXXFLAGS) -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: clean
