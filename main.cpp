// Satellite view with drone image overlay using OpenGL + Dear ImGui
// Uses ODM images.json + cameras.json for accurate positioning

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <curl/curl.h>

#include "imgui_lib/imgui.h"
#include "imgui_lib/backends/imgui_impl_glfw.h"
#include "imgui_lib/backends/imgui_impl_opengl3.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <filesystem>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <fstream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

// Simple JSON value parser (minimal, for our specific JSON files)
#include <sstream>

// ============ Constants ============
static const double PI = 3.14159265358979323846;
static const int TILE_SIZE = 256;
static const int INITIAL_ZOOM = 18;
static const int WIN_W = 1280, WIN_H = 900;
static const int DRONE_TEX_MAX = 512;

// Reference zoom used to express alignment corrections in mosaic-pixel space.
static const int ALIGN_REF_ZOOM = 19;

// ORB / matching parameters
static const int   ORB_NFEATURES        = 3000;
static const int   ALIGN_NEIGHBORS      = 4;     // up to N previous images to match against
static const int   ALIGN_MIN_INLIERS    = 12;    // below this, fallback to GPS
static const float ALIGN_RATIO          = 0.80f; // Lowe ratio test (looser)
static const double ALIGN_RANSAC_METERS = 2.0;   // RANSAC inlier threshold in meters

// ============ Math helpers ============
double degToRad(double d) { return d * PI / 180.0; }
double radToDeg(double r) { return r * 180.0 / PI; }

double lon2tilex(double lon, int z) { return (lon + 180.0) / 360.0 * (1 << z); }
double lat2tiley(double lat, int z) {
    double latrad = degToRad(lat);
    return (1.0 - std::log(std::tan(latrad) + 1.0 / std::cos(latrad)) / PI) / 2.0 * (1 << z);
}
double tilex2lon(double tx, int z) { return tx / (double)(1 << z) * 360.0 - 180.0; }
double tiley2lat(double ty, int z) {
    double n = PI - 2.0 * PI * ty / (double)(1 << z);
    return radToDeg(std::atan(0.5 * (std::exp(n) - std::exp(-n))));
}

double metersPerPixel(double lat, int z) {
    return 156543.03392 * std::cos(degToRad(lat)) / (1 << z);
}

// ============ Drone image metadata ============
struct DroneImage {
    std::string path;
    std::string filename;
    double lat, lon, alt;
    double yaw, pitch, roll;
    double omega, phi, kappa;  // ODM orientation angles
    double focalX;             // normalized focal length (from cameras.json)
    double cx, cy;             // principal point offset (normalized)
    int imgW, imgH;
    GLuint tex;
    bool texLoaded;
    unsigned char *cpuPixels;
    int cpuW, cpuH;
    bool cpuLoaded;
    bool visible;
    float yawAdjust;

    // ORB features (in resized cpuW x cpuH pixel coords)
    std::vector<cv::KeyPoint> kpts;
    cv::Mat descriptors;

    // Alignment result (corrects the GPS prediction)
    double correctedLat, correctedLon;  // refined center
    double correctedYaw;                // degrees, CW from north
    double correctedScale;              // multiplier on GPS-predicted footprint size
    int    alignInliers;                // 0 = not refined / failed
    bool   alignAnchor;                 // true for image 0 — placed by GPS, no refinement
};

// ============ Simple JSON helpers ============
// Extract a double value for a given key from a JSON string
static double jsonGetDouble(const std::string &json, const std::string &key) {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search);
    if (pos == std::string::npos) return 0;
    pos = json.find(':', pos);
    if (pos == std::string::npos) return 0;
    pos++;
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    return atof(json.c_str() + pos);
}

static int jsonGetInt(const std::string &json, const std::string &key) {
    return (int)jsonGetDouble(json, key);
}

static std::string jsonGetString(const std::string &json, const std::string &key) {
    std::string search = "\"" + key + "\"";
    auto pos = json.find(search);
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos);
    if (pos == std::string::npos) return "";
    pos = json.find('"', pos);
    if (pos == std::string::npos) return "";
    pos++;
    auto end = json.find('"', pos);
    if (end == std::string::npos) return "";
    return json.substr(pos, end - pos);
}

// Read the entire file into a string
static std::string readFile(const std::string &path) {
    std::ifstream f(path);
    if (!f.is_open()) return "";
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Parse images.json - it's an array of objects
void readImagesJson(const std::string &odmDir, const std::string &imageDir,
                    double camFocalX, double camCx, double camCy,
                    std::vector<DroneImage> &out) {
    std::string json = readFile(odmDir + "/images.json");
    if (json.empty()) return;

    // Split by objects (find each { ... })
    size_t pos = 0;
    while (pos < json.size()) {
        auto start = json.find('{', pos);
        if (start == std::string::npos) break;
        // Find matching closing brace
        int depth = 0;
        size_t end = start;
        for (; end < json.size(); end++) {
            if (json[end] == '{') depth++;
            if (json[end] == '}') { depth--; if (depth == 0) break; }
        }
        if (end >= json.size()) break;

        std::string obj = json.substr(start, end - start + 1);
        pos = end + 1;

        std::string filename = jsonGetString(obj, "filename");
        if (filename.empty()) continue;

        DroneImage di = {};
        di.filename = filename;
        di.path = imageDir + "/" + filename;
        di.lat = jsonGetDouble(obj, "latitude");
        di.lon = jsonGetDouble(obj, "longitude");
        di.alt = jsonGetDouble(obj, "altitude");
        di.yaw = jsonGetDouble(obj, "yaw");
        di.pitch = jsonGetDouble(obj, "pitch");
        di.roll = jsonGetDouble(obj, "roll");
        di.omega = jsonGetDouble(obj, "omega");
        di.phi = jsonGetDouble(obj, "phi");
        di.kappa = jsonGetDouble(obj, "kappa");
        di.focalX = camFocalX;
        di.cx = camCx;
        di.cy = camCy;
        di.imgW = jsonGetInt(obj, "width");
        di.imgH = jsonGetInt(obj, "height");
        di.tex = 0;
        di.texLoaded = false;
        di.cpuPixels = nullptr;
        di.cpuLoaded = false;
        di.visible = true;
        di.yawAdjust = 0;
        di.correctedLat = di.lat;
        di.correctedLon = di.lon;
        di.correctedYaw = di.yaw;
        di.correctedScale = 1.0;
        di.alignInliers = 0;
        di.alignAnchor = false;

        if (di.lat != 0 && di.lon != 0) {
            out.push_back(di);
        }
    }
}

// Parse cameras.json to get focal_x, c_x, c_y
void readCamerasJson(const std::string &odmDir, double &focalX, double &cx, double &cy) {
    std::string json = readFile(odmDir + "/cameras.json");
    if (json.empty()) return;
    // There's only one camera entry, just find the values
    focalX = jsonGetDouble(json, "focal_x");
    cx = jsonGetDouble(json, "c_x");
    cy = jsonGetDouble(json, "c_y");
}

// ============ Curl download ============
static size_t curlWriteCallback(void *ptr, size_t size, size_t nmemb, void *userdata) {
    auto *buf = (std::vector<unsigned char>*)userdata;
    size_t total = size * nmemb;
    buf->insert(buf->end(), (unsigned char*)ptr, (unsigned char*)ptr + total);
    return total;
}

std::vector<unsigned char> downloadTile(int z, int y, int x) {
    std::vector<unsigned char> data;
    char url[512];
    snprintf(url, sizeof(url),
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%d/%d", z, y, x);
    CURL *curl = curl_easy_init();
    if (!curl) return data;
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "DroneOverlay/1.0");
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    return data;
}

// ============ Async tile system ============
struct TileKey {
    int z, y, x;
    bool operator<(const TileKey &o) const {
        if (z != o.z) return z < o.z;
        if (y != o.y) return y < o.y;
        return x < o.x;
    }
};

enum TileState { TILE_EMPTY, TILE_LOADING, TILE_READY, TILE_UPLOADED, TILE_FAILED };

struct TileData {
    GLuint tex;
    TileState state;
    unsigned char *pixels;
    int w, h;
};

static std::map<TileKey, TileData> tileCache;
static std::mutex tileMutex;
static std::queue<TileKey> tileRequestQueue;
static std::mutex requestMutex;
static std::atomic<bool> tileThreadRunning{true};

void tileWorker() {
    while (tileThreadRunning) {
        TileKey key;
        bool hasWork = false;
        {
            std::lock_guard<std::mutex> lk(requestMutex);
            if (!tileRequestQueue.empty()) {
                key = tileRequestQueue.front();
                tileRequestQueue.pop();
                hasWork = true;
            }
        }
        if (!hasWork) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto data = downloadTile(key.z, key.y, key.x);
        if (!data.empty()) {
            int w, h, ch;
            unsigned char *img = stbi_load_from_memory(data.data(), (int)data.size(), &w, &h, &ch, 4);
            if (img) {
                std::lock_guard<std::mutex> lk(tileMutex);
                auto &td = tileCache[key];
                td.pixels = img;
                td.w = w;
                td.h = h;
                td.state = TILE_READY;
            } else {
                std::lock_guard<std::mutex> lk(tileMutex);
                tileCache[key].state = TILE_FAILED;
            }
        } else {
            std::lock_guard<std::mutex> lk(tileMutex);
            tileCache[key].state = TILE_FAILED;
        }
    }
}

GLuint requestTile(int z, int y, int x) {
    TileKey key = {z, y, x};
    std::lock_guard<std::mutex> lk(tileMutex);
    auto it = tileCache.find(key);
    if (it != tileCache.end()) {
        auto &td = it->second;
        if (td.state == TILE_UPLOADED) return td.tex;
        if (td.state == TILE_READY) {
            glGenTextures(1, &td.tex);
            glBindTexture(GL_TEXTURE_2D, td.tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, td.w, td.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, td.pixels);
            stbi_image_free(td.pixels);
            td.pixels = nullptr;
            td.state = TILE_UPLOADED;
            return td.tex;
        }
        return 0;
    }

    TileData td = {};
    td.state = TILE_LOADING;
    tileCache[key] = td;
    {
        std::lock_guard<std::mutex> rlk(requestMutex);
        tileRequestQueue.push(key);
    }
    return 0;
}

// ============ View state ============
struct ViewState {
    double centerLat, centerLon;
    int zoom;
    double offsetX, offsetY;
    bool dragging;
    double dragStartX, dragStartY;
    double dragOrigOffX, dragOrigOffY;
    float droneOpacity;
    bool showDrones;
    float groundElevation;
    float yawOffset;
    float scaleMultiplier;
    int selectedImage;
    int maxVisibleImages;
    int totalImages;
    float maxPitch; // filter out oblique images
    bool useAlignment;       // toggle ORB-based refinement vs raw GPS
    bool showAlignStatus;    // colored borders by alignment status
};

static ViewState view;
static std::vector<DroneImage> droneImages;

// ============ Main ============
int main(int argc, char **argv) {
    std::string imageDir = "images";
    std::string odmDir = "";

    // Auto-detect ODM directory
    for (auto &entry : std::filesystem::directory_iterator(".")) {
        if (entry.is_directory()) {
            std::string name = entry.path().filename().string();
            if (name.find("odm") != std::string::npos || name.find("ODM") != std::string::npos) {
                if (std::filesystem::exists(entry.path() / "images.json") &&
                    std::filesystem::exists(entry.path() / "cameras.json")) {
                    odmDir = entry.path().string();
                    printf("Found ODM directory: %s\n", odmDir.c_str());
                    break;
                }
            }
        }
    }

    if (argc > 1) imageDir = argv[1];
    if (argc > 2) odmDir = argv[2];

    // Read camera calibration from ODM
    double camFocalX = 0, camCx = 0, camCy = 0;
    if (!odmDir.empty()) {
        readCamerasJson(odmDir, camFocalX, camCx, camCy);
        printf("Camera: focal_x=%.6f c_x=%.6f c_y=%.6f\n", camFocalX, camCx, camCy);
    }

    // Read image metadata
    printf("Reading image metadata...\n");
    if (!odmDir.empty()) {
        readImagesJson(odmDir, imageDir, camFocalX, camCx, camCy, droneImages);
        printf("Loaded %zu images from ODM images.json\n", droneImages.size());
    }

    if (droneImages.empty()) {
        fprintf(stderr, "No images found. Provide ODM directory with images.json\n");
        return 1;
    }

    int totalImages = (int)droneImages.size();
    int maxVisibleImages = std::min(3, totalImages); // start with 3, adjustable via slider

    for (auto &di : droneImages) {
        printf("  %s: lat=%.6f lon=%.6f alt=%.1f yaw=%.1f kappa=%.1f\n",
            di.filename.c_str(), di.lat, di.lon, di.alt, di.yaw, di.kappa);
    }

    double avgLat = 0, avgLon = 0;
    for (auto &d : droneImages) { avgLat += d.lat; avgLon += d.lon; }
    avgLat /= droneImages.size();
    avgLon /= droneImages.size();

    view.centerLat = avgLat;
    view.centerLon = avgLon;
    view.zoom = INITIAL_ZOOM;
    view.offsetX = view.offsetY = 0;
    view.dragging = false;
    view.droneOpacity = 0.6f;
    view.showDrones = true;
    view.groundElevation = 240.0f;
    view.yawOffset = 0.0f;
    view.scaleMultiplier = 1.0f;
    view.selectedImage = -1;
    view.maxVisibleImages = maxVisibleImages;
    view.totalImages = totalImages;
    view.maxPitch = 10.0f;
    view.useAlignment = true;
    view.showAlignStatus = true;

    // Init GLFW
    if (!glfwInit()) { fprintf(stderr, "GLFW init failed\n"); return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWwindow *window = glfwCreateWindow(WIN_W, WIN_H, "Satellite + Drone Overlay", NULL, NULL);
    if (!window) { fprintf(stderr, "Window creation failed\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glewInit();

    // Init ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.Alpha = 0.95f;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);

    // Start tile download threads
    std::vector<std::thread> tileThreads;
    for (int i = 0; i < 4; i++) {
        tileThreads.emplace_back(tileWorker);
    }

    // ============ Alignment helpers ============
    // Predicted footprint (mosaic-px at ALIGN_REF_ZOOM) using current corrected pose.
    auto poseFootprint = [&](const DroneImage &d, double &cxPx, double &cyPx,
                             double &fpW, double &fpH, double &yawDeg) {
        double altAGL = d.alt - view.groundElevation;
        if (altAGL < 5) altAGL = 5;
        double fx = d.focalX > 0 ? d.focalX : 0.7413;
        double groundW = altAGL / fx;
        double aspect = d.imgW > 0 ? (double)d.imgH / d.imgW : 0.75;
        double groundH = groundW * aspect;
        double mpp = metersPerPixel(d.correctedLat, ALIGN_REF_ZOOM);
        fpW = (groundW / mpp) * d.correctedScale;
        fpH = (groundH / mpp) * d.correctedScale;
        cxPx = lon2tilex(d.correctedLon, ALIGN_REF_ZOOM) * TILE_SIZE;
        cyPx = lat2tiley(d.correctedLat, ALIGN_REF_ZOOM) * TILE_SIZE;
        yawDeg = d.correctedYaw;
    };

    // Map an image-pixel keypoint (u,v in [0,W)x[0,H)) to mosaic-px coords given a pose.
    // Mirrors the GL transform: T(center) * R(yaw) * T(-ppOff) * v_local
    auto imgPtToMosaic = [&](const DroneImage &d, double cxPx, double cyPx,
                             double fpW, double fpH, double yawDeg,
                             double u, double v) {
        double localU = (u / d.cpuW - 0.5) * fpW;
        double localV = (v / d.cpuH - 0.5) * fpH;
        double ppOffX = d.cx * fpW;
        double ppOffY = d.cy * fpH;
        localU -= ppOffX;
        localV -= ppOffY;
        double a = degToRad(yawDeg);
        double ca = std::cos(a), sa = std::sin(a);
        double rx = ca * localU - sa * localV;
        double ry = sa * localU + ca * localV;
        return cv::Point2d(cxPx + rx, cyPx + ry);
    };

    // Background drone texture loader — loads images as maxVisibleImages increases
    std::thread droneLoader([&]() {
        cv::Ptr<cv::ORB> orb = cv::ORB::create(ORB_NFEATURES);
        cv::BFMatcher matcher(cv::NORM_HAMMING, false);

        while (tileThreadRunning) {
            bool didWork = false;
            int limit = view.maxVisibleImages;
            for (int i = 0; i < limit && i < (int)droneImages.size(); i++) {
                auto &di = droneImages[i];
                if (di.cpuLoaded || !tileThreadRunning) continue;
                didWork = true;

                int w, h, ch;
                unsigned char *img = stbi_load(di.path.c_str(), &w, &h, &ch, 4);
                if (!img) {
                    fprintf(stderr, "[drone] stbi_load failed: %s (%s)\n",
                            di.path.c_str(), stbi_failure_reason());
                    di.cpuLoaded = true;
                    continue;
                }

                int newW = w, newH = h;
                if (w > DRONE_TEX_MAX || h > DRONE_TEX_MAX) {
                    float sc = (float)DRONE_TEX_MAX / std::max(w, h);
                    newW = (int)(w * sc);
                    newH = (int)(h * sc);
                    unsigned char *resized = (unsigned char*)malloc(newW * newH * 4);
                    stbir_resize_uint8_linear(img, w, h, 0, resized, newW, newH, 0, (stbir_pixel_layout)4);
                    stbi_image_free(img);
                    img = resized;
                }

                if (di.imgW == 0) di.imgW = w;
                if (di.imgH == 0) di.imgH = h;
                di.cpuPixels = img;
                di.cpuW = newW;
                di.cpuH = newH;

                // ---- ORB extraction on grayscale of the resized image ----
                cv::Mat rgba(newH, newW, CV_8UC4, di.cpuPixels);
                cv::Mat gray;
                cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
                orb->detectAndCompute(gray, cv::noArray(), di.kpts, di.descriptors);

                // ---- Alignment: refine pose against earlier images ----
                if (i == 0) {
                    di.alignAnchor = true;
                } else if (!di.descriptors.empty()) {
                    // Predict pose for `di` from raw GPS (correctedLat/Lon/Yaw/Scale init = GPS)
                    double cxA, cyA, fpWA, fpHA, yawA;
                    poseFootprint(di, cxA, cyA, fpWA, fpHA, yawA);

                    // RANSAC threshold expressed in mosaic-px at REF_ZOOM
                    double mppRef = metersPerPixel(di.lat, ALIGN_REF_ZOOM);
                    double ransacThresh = ALIGN_RANSAC_METERS / mppRef;

                    // Pick up to ALIGN_NEIGHBORS earlier images that have features.
                    // Allow GPS-fallback neighbors too — their pose is still close enough to seed.
                    struct Cand { int idx; double dist; };
                    std::vector<Cand> cands;
                    cands.reserve(i);
                    for (int j = 0; j < i; j++) {
                        const auto &dj = droneImages[j];
                        if (!dj.cpuLoaded || dj.descriptors.empty()) continue;
                        double dlat = dj.correctedLat - di.lat;
                        double dlon = dj.correctedLon - di.lon;
                        cands.push_back({j, dlat*dlat + dlon*dlon});
                    }
                    std::sort(cands.begin(), cands.end(),
                              [](const Cand &a, const Cand &b){ return a.dist < b.dist; });
                    if ((int)cands.size() > ALIGN_NEIGHBORS) cands.resize(ALIGN_NEIGHBORS);

                    // Accumulate matched mosaic-px point pairs across neighbors.
                    std::vector<cv::Point2d> srcPts; // in di's predicted mosaic coords
                    std::vector<cv::Point2d> dstPts; // in neighbor's corrected mosaic coords
                    int totalRaw = 0, totalRatio = 0;
                    for (const auto &c : cands) {
                        const auto &dj = droneImages[c.idx];
                        std::vector<std::vector<cv::DMatch>> knn;
                        matcher.knnMatch(di.descriptors, dj.descriptors, knn, 2);
                        totalRaw += (int)knn.size();

                        double cxB, cyB, fpWB, fpHB, yawB;
                        poseFootprint(dj, cxB, cyB, fpWB, fpHB, yawB);

                        int kept = 0;
                        for (auto &m : knn) {
                            if (m.size() < 2) continue;
                            if (m[0].distance > ALIGN_RATIO * m[1].distance) continue;
                            const auto &kpA = di.kpts[m[0].queryIdx].pt;
                            const auto &kpB = dj.kpts[m[0].trainIdx].pt;
                            srcPts.push_back(imgPtToMosaic(di, cxA, cyA, fpWA, fpHA, yawA,
                                                           kpA.x, kpA.y));
                            dstPts.push_back(imgPtToMosaic(dj, cxB, cyB, fpWB, fpHB, yawB,
                                                           kpB.x, kpB.y));
                            kept++;
                        }
                        totalRatio += kept;
                    }

                    if (srcPts.size() >= (size_t)ALIGN_MIN_INLIERS) {
                        std::vector<uchar> inliersMask;
                        cv::Mat M = cv::estimateAffinePartial2D(
                            srcPts, dstPts, inliersMask, cv::RANSAC, ransacThresh);
                        int inliers = 0;
                        if (!M.empty()) {
                            for (auto v : inliersMask) if (v) inliers++;
                        }
                        if (inliers >= ALIGN_MIN_INLIERS) {
                            double a  = M.at<double>(0, 0);
                            double bb = M.at<double>(1, 0);
                            double tx = M.at<double>(0, 2);
                            double ty = M.at<double>(1, 2);
                            double scaleCorr = std::sqrt(a*a + bb*bb);
                            double rotCorr   = std::atan2(bb, a);

                            double newCx = a * cxA - bb * cyA + tx;
                            double newCy = bb * cxA +  a * cyA + ty;

                            di.correctedLon = tilex2lon(newCx / TILE_SIZE, ALIGN_REF_ZOOM);
                            di.correctedLat = tiley2lat(newCy / TILE_SIZE, ALIGN_REF_ZOOM);
                            di.correctedYaw = di.yaw + radToDeg(rotCorr);
                            di.correctedScale = scaleCorr;
                            di.alignInliers = inliers;
                            fprintf(stderr,
                                "[align] %s OK  cands=%zu raw=%d ratio=%d inliers=%d  scale=%.3f rot=%+.2f\n",
                                di.filename.c_str(), cands.size(), totalRaw, totalRatio,
                                inliers, scaleCorr, radToDeg(rotCorr));
                        } else {
                            fprintf(stderr,
                                "[align] %s FAIL inliers cands=%zu raw=%d ratio=%d inliers=%d (thresh=%.1fpx=%.1fm)\n",
                                di.filename.c_str(), cands.size(), totalRaw, totalRatio,
                                inliers, ransacThresh, ALIGN_RANSAC_METERS);
                        }
                    } else {
                        fprintf(stderr,
                            "[align] %s FAIL matches cands=%zu raw=%d ratio=%d (need %d)\n",
                            di.filename.c_str(), cands.size(), totalRaw, totalRatio,
                            ALIGN_MIN_INLIERS);
                    }
                }

                di.cpuLoaded = true;
            }
            if (!didWork)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int winW, winH;
        glfwGetFramebufferSize(window, &winW, &winH);

        // Handle mouse drag for panning (only when not over ImGui)
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                view.dragging = true;
                view.dragStartX = io.MousePos.x;
                view.dragStartY = io.MousePos.y;
                view.dragOrigOffX = view.offsetX;
                view.dragOrigOffY = view.offsetY;
            }
            if (view.dragging && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                view.offsetX = view.dragOrigOffX + (io.MousePos.x - view.dragStartX);
                view.offsetY = view.dragOrigOffY + (io.MousePos.y - view.dragStartY);
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                view.dragging = false;
            }

            if (io.MouseWheel != 0) {
                int oldZoom = view.zoom;
                view.zoom += (int)io.MouseWheel;
                view.zoom = std::clamp(view.zoom, 1, 20);
                if (view.zoom != oldZoom) {
                    double scale = std::pow(2.0, view.zoom - oldZoom);
                    view.offsetX *= scale;
                    view.offsetY *= scale;
                }
            }
        }

        if (!io.WantCaptureKeyboard) {
            if (ImGui::IsKeyPressed(ImGuiKey_Escape))
                glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        // ============ OpenGL rendering ============
        glViewport(0, 0, winW, winH);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, winW, winH, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        int z = view.zoom;
        double cx = lon2tilex(view.centerLon, z);
        double cy = lat2tiley(view.centerLat, z);
        double centerPixX = cx * TILE_SIZE;
        double centerPixY = cy * TILE_SIZE;
        double screenCX = winW / 2.0 + view.offsetX;
        double screenCY = winH / 2.0 + view.offsetY;

        int tileMinX = (int)std::floor((centerPixX - screenCX) / TILE_SIZE);
        int tileMaxX = (int)std::floor((centerPixX + (winW - screenCX)) / TILE_SIZE);
        int tileMinY = (int)std::floor((centerPixY - screenCY) / TILE_SIZE);
        int tileMaxY = (int)std::floor((centerPixY + (winH - screenCY)) / TILE_SIZE);
        int maxTile = (1 << z) - 1;

        // Draw satellite tiles
        glColor4f(1, 1, 1, 1);
        for (int ty = tileMinY; ty <= tileMaxY; ty++) {
            for (int tx = tileMinX; tx <= tileMaxX; tx++) {
                if (tx < 0 || ty < 0 || tx > maxTile || ty > maxTile) continue;
                GLuint tex = requestTile(z, ty, tx);
                if (!tex) continue;

                double sx = tx * TILE_SIZE - centerPixX + screenCX;
                double sy = ty * TILE_SIZE - centerPixY + screenCY;

                glBindTexture(GL_TEXTURE_2D, tex);
                glBegin(GL_QUADS);
                glTexCoord2f(0, 0); glVertex2d(sx, sy);
                glTexCoord2f(1, 0); glVertex2d(sx + TILE_SIZE, sy);
                glTexCoord2f(1, 1); glVertex2d(sx + TILE_SIZE, sy + TILE_SIZE);
                glTexCoord2f(0, 1); glVertex2d(sx, sy + TILE_SIZE);
                glEnd();
            }
        }

        // Upload up to 2 drone textures per frame
        int uploaded = 0;
        for (auto &di : droneImages) {
            if (uploaded >= 2) break;
            if (di.texLoaded || !di.cpuLoaded) continue;
            if (!di.cpuPixels || di.cpuW <= 0 || di.cpuH <= 0) {
                di.texLoaded = true; // failed load, don't retry
                continue;
            }
            glGenTextures(1, &di.tex);
            glBindTexture(GL_TEXTURE_2D, di.tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, di.cpuW, di.cpuH, 0, GL_RGBA, GL_UNSIGNED_BYTE, di.cpuPixels);
            free(di.cpuPixels);
            di.cpuPixels = nullptr;
            di.texLoaded = true;
            uploaded++;
        }

        // Draw drone images overlay
        if (view.showDrones) {
            for (int i = 0; i < view.maxVisibleImages && i < (int)droneImages.size(); i++) {
                auto &di = droneImages[i];
                if (!di.texLoaded || !di.visible) continue;
                if (std::abs(di.pitch) > view.maxPitch) continue;

                bool useRefined = view.useAlignment;
                double useLat   = useRefined ? di.correctedLat   : di.lat;
                double useLon   = useRefined ? di.correctedLon   : di.lon;
                double useYaw   = useRefined ? di.correctedYaw   : di.yaw;
                double useScale = useRefined ? di.correctedScale : 1.0;

                double dx = lon2tilex(useLon, z) * TILE_SIZE;
                double dy = lat2tiley(useLat, z) * TILE_SIZE;
                double screenX = dx - centerPixX + screenCX;
                double screenY = dy - centerPixY + screenCY;

                if (screenX < -1000 || screenX > winW + 1000 || screenY < -1000 || screenY > winH + 1000)
                    continue;

                // Ground footprint using ODM normalized focal length
                double altAGL = di.alt - view.groundElevation;
                if (altAGL < 5) altAGL = 5;
                double fx = di.focalX > 0 ? di.focalX : 0.7413;
                double groundW = altAGL / fx;  // meters
                double aspectRatio = (double)di.imgH / di.imgW;
                double groundH = groundW * aspectRatio;

                groundW *= view.scaleMultiplier * useScale;
                groundH *= view.scaleMultiplier * useScale;

                double mpp = metersPerPixel(useLat, z);
                double pixW = groundW / mpp;
                double pixH = groundH / mpp;

                float totalRot = (float)useYaw + view.yawOffset + di.yawAdjust;

                bool isSelected = (view.selectedImage == i);
                float alpha = isSelected ? std::min(1.0f, view.droneOpacity + 0.2f) : view.droneOpacity;

                double ppOffX = di.cx * pixW;
                double ppOffY = di.cy * pixH;

                glPushMatrix();
                glTranslated(screenX, screenY, 0);
                glRotatef(totalRot, 0, 0, 1);
                glTranslated(-ppOffX, -ppOffY, 0);

                glBindTexture(GL_TEXTURE_2D, di.tex);
                glColor4f(1, 1, 1, alpha);
                glBegin(GL_QUADS);
                glTexCoord2f(0, 0); glVertex2d(-pixW/2, -pixH/2);
                glTexCoord2f(1, 0); glVertex2d( pixW/2, -pixH/2);
                glTexCoord2f(1, 1); glVertex2d( pixW/2,  pixH/2);
                glTexCoord2f(0, 1); glVertex2d(-pixW/2,  pixH/2);
                glEnd();

                // Border: yellow if selected; otherwise color-code alignment status if enabled.
                bool drawBorder = isSelected || view.showAlignStatus;
                if (drawBorder) {
                    glBindTexture(GL_TEXTURE_2D, 0);
                    if (isSelected) {
                        glColor4f(1, 1, 0, 1);
                    } else if (di.alignAnchor) {
                        glColor4f(0.2f, 0.6f, 1.0f, 0.7f); // blue: anchor
                    } else if (di.alignInliers >= ALIGN_MIN_INLIERS) {
                        glColor4f(0.2f, 1.0f, 0.2f, 0.7f); // green: aligned
                    } else {
                        glColor4f(1.0f, 0.3f, 0.3f, 0.7f); // red: GPS fallback
                    }
                    glLineWidth(isSelected ? 2.5f : 1.5f);
                    glBegin(GL_LINE_LOOP);
                    glVertex2d(-pixW/2, -pixH/2);
                    glVertex2d( pixW/2, -pixH/2);
                    glVertex2d( pixW/2,  pixH/2);
                    glVertex2d(-pixW/2,  pixH/2);
                    glEnd();
                }

                glPopMatrix();
            }
        }

        // ============ ImGui panels ============

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Controls");

        ImGui::SeparatorText("View");
        ImGui::Text("Zoom: %d", view.zoom);
        ImGui::Text("Center: %.5f, %.5f", view.centerLat, view.centerLon);
        if (ImGui::Button("Reset View")) {
            view.offsetX = view.offsetY = 0;
            view.zoom = INITIAL_ZOOM;
        }

        ImGui::SeparatorText("Drone Overlay");
        ImGui::Checkbox("Show Drones", &view.showDrones);
        ImGui::SliderInt("Nb Images", &view.maxVisibleImages, 0, view.totalImages);
        ImGui::SliderFloat("Max Pitch", &view.maxPitch, 0.0f, 15.0f, "%.0f deg");
        ImGui::SetItemTooltip("Filter out oblique images (high pitch = not pointing down)");
        ImGui::SliderFloat("Opacity", &view.droneOpacity, 0.0f, 1.0f);

        ImGui::SeparatorText("Calibration");
        ImGui::SliderFloat("Ground Elev. (m)", &view.groundElevation, 100.0f, 500.0f, "%.0f m");
        ImGui::SetItemTooltip("Altitude of the terrain above sea level");
        ImGui::SliderFloat("Scale", &view.scaleMultiplier, 0.1f, 5.0f, "%.2f");
        ImGui::SliderFloat("Rotation Offset", &view.yawOffset, -180.0f, 180.0f, "%.1f deg");
        ImGui::SetItemTooltip("Global rotation offset applied to all images");

        if (ImGui::Button("Reset Calibration")) {
            view.groundElevation = 240.0f;
            view.scaleMultiplier = 1.0f;
            view.yawOffset = 0.0f;
            view.maxPitch = 10.0f;
        }

        ImGui::SeparatorText("Alignment (ORB)");
        ImGui::Checkbox("Use ORB refinement", &view.useAlignment);
        ImGui::SetItemTooltip("Off = raw GPS placement. On = each image refined against earlier neighbors.");
        ImGui::Checkbox("Show status borders", &view.showAlignStatus);
        ImGui::SetItemTooltip("Blue=anchor, green=aligned, red=GPS fallback");

        ImGui::SeparatorText("Info");
        int loadedCount = 0;
        int alignedCount = 0;
        int fallbackCount = 0;
        for (int i = 0; i < view.maxVisibleImages && i < (int)droneImages.size(); i++) {
            const auto &dii = droneImages[i];
            if (dii.texLoaded) loadedCount++;
            if (dii.cpuLoaded) {
                if (dii.alignAnchor) {} // anchor not counted as "aligned"
                else if (dii.alignInliers >= ALIGN_MIN_INLIERS) alignedCount++;
                else fallbackCount++;
            }
        }
        ImGui::Text("Images loaded: %d / %d (total: %d)", loadedCount, view.maxVisibleImages, view.totalImages);
        ImGui::Text("Aligned: %d   GPS fallback: %d", alignedCount, fallbackCount);
        ImGui::Text("FPS: %.0f", io.Framerate);

        ImGui::End();

        // Image list panel
        ImGui::SetNextWindowPos(ImVec2(10, 420), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(340, 350), ImGuiCond_FirstUseEver);
        ImGui::Begin("Images");

        for (int i = 0; i < (int)droneImages.size(); i++) {
            auto &di = droneImages[i];
            ImGui::PushID(i);

            bool isSelected = (view.selectedImage == i);
            if (isSelected) ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.8f, 0.6f, 0.0f, 0.5f));

            if (ImGui::CollapsingHeader(di.filename.c_str())) {
                ImGui::Checkbox("Visible", &di.visible);
                ImGui::Text("Lat: %.6f  Lon: %.6f", di.lat, di.lon);
                ImGui::Text("Alt: %.1f m (AGL: %.1f m)", di.alt, di.alt - view.groundElevation);
                ImGui::Text("Yaw: %.1f  Pitch: %.1f  Roll: %.1f", di.yaw, di.pitch, di.roll);
                ImGui::Text("Kappa: %.1f  Omega: %.1f  Phi: %.1f", di.kappa, di.omega, di.phi);
                ImGui::Text("Focal_x: %.4f  Size: %d x %d", di.focalX, di.imgW, di.imgH);

                if (di.alignAnchor) {
                    ImGui::TextColored(ImVec4(0.4f,0.7f,1,1), "Alignment: anchor (GPS)");
                } else if (di.alignInliers >= ALIGN_MIN_INLIERS) {
                    ImGui::TextColored(ImVec4(0.4f,1,0.4f,1),
                        "Aligned: inliers=%d  scale=%.3f  dYaw=%.2f deg",
                        di.alignInliers, di.correctedScale, di.correctedYaw - di.yaw);
                } else if (di.cpuLoaded) {
                    ImGui::TextColored(ImVec4(1,0.4f,0.4f,1), "Alignment: GPS fallback");
                } else {
                    ImGui::TextDisabled("Alignment: pending...");
                }

                ImGui::Separator();
                ImGui::SliderFloat("Rot. Adjust", &di.yawAdjust, -180.0f, 180.0f, "%.1f");
                if (ImGui::Button("Reset Adj.")) {
                    di.yawAdjust = 0;
                }

                ImGui::Separator();
                if (ImGui::Button(isSelected ? "Deselect" : "Select")) {
                    view.selectedImage = isSelected ? -1 : i;
                }
                ImGui::SameLine();
                if (ImGui::Button("Center On")) {
                    double newCX = lon2tilex(di.lon, z) * TILE_SIZE;
                    double newCY = lat2tiley(di.lat, z) * TILE_SIZE;
                    view.offsetX = -(newCX - cx * TILE_SIZE);
                    view.offsetY = -(newCY - cy * TILE_SIZE);
                }
            }

            if (isSelected) ImGui::PopStyleColor();
            ImGui::PopID();
        }

        ImGui::End();

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    tileThreadRunning = false;
    for (auto &t : tileThreads) t.join();
    if (droneLoader.joinable()) droneLoader.join();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    {
        std::lock_guard<std::mutex> lk(tileMutex);
        for (auto &[k, v] : tileCache) {
            if (v.tex) glDeleteTextures(1, &v.tex);
            if (v.pixels) stbi_image_free(v.pixels);
        }
    }
    for (auto &di : droneImages) {
        if (di.tex) glDeleteTextures(1, &di.tex);
        if (di.cpuPixels) free(di.cpuPixels);
    }

    curl_global_cleanup();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
