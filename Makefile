CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-unused-result

UNAME_S := $(shell uname -s)
ifneq (,$(findstring MINGW,$(UNAME_S))$(findstring MSYS,$(UNAME_S))$(findstring CYGWIN,$(UNAME_S))$(findstring UCRT,$(UNAME_S)))
    LDFLAGS = -lglfw3 -lglew32 -lopengl32 -lcurl
else
    LDFLAGS = -lglfw -lGLEW -lGL -lcurl
endif

# OpenCV (used for ORB-based image alignment)
OPENCV_CFLAGS  := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv 2>/dev/null)
OPENCV_LDFLAGS := $(shell pkg-config --libs   opencv4 2>/dev/null || pkg-config --libs   opencv 2>/dev/null)
ifeq ($(strip $(OPENCV_CFLAGS)),)
    # Fallback: try common MSYS2 / Linux include paths
    ifneq ($(wildcard /ucrt64/include/opencv4/opencv2/core.hpp),)
        OPENCV_CFLAGS := -I/ucrt64/include/opencv4
    else ifneq ($(wildcard /mingw64/include/opencv4/opencv2/core.hpp),)
        OPENCV_CFLAGS := -I/mingw64/include/opencv4
    else ifneq ($(wildcard /usr/include/opencv4/opencv2/core.hpp),)
        OPENCV_CFLAGS := -I/usr/include/opencv4
    endif
endif
ifeq ($(strip $(OPENCV_LDFLAGS)),)
    OPENCV_LDFLAGS := -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_calib3d
endif

CXXFLAGS += $(OPENCV_CFLAGS)
LDFLAGS  += $(OPENCV_LDFLAGS)

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
