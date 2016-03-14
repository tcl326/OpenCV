#include "opencv2/video/tracking.hpp"
// Video write
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videostab/videostab.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include <ctype.h>
#include <cstdlib>
#include <list>
#include <iterator>
#include "Miniball.hpp"
#include <math.h>
#include <typeinfo>
#include "JenksNaturalBreak.hpp"
#include <fstream>
#include <ratio>
#include <chrono>

using namespace cv;
using namespace std;
using namespace cv::videostab;
using namespace std::chrono;

bool testing = true;
bool profiling = false;

high_resolution_clock::time_point start;
high_resolution_clock::time_point ending;
duration<double> timeSpan;

ofstream report;


vector<vector<Point2f>> storedPoints {{{52.44, 122.95}, {38.18, 53.96}, {76.92, 18.15}, {129.91, 50.33}, {26.69, 93.97}, {145.53, 5.47}, {58.25, 112.1}, {96.62, 52.11}, {88.08, 83.77}, {27.61, 56.25}, {23.35, 18.68}, {129.95, 77.76}, {77.67, 124.39}, {98.74, 143.31}, {55.56, 63.01}, {88.72, 5.6}, {100.94, 139.84}, {37.55, 97.89}, {114.07, 30.15}, {1.93, 77.69}, {6.8, 84.55}, {68.55, 75.06}, {9.63, 35.02}, {16.76, 131.08}, {32.49, 23.31}, {28.44, 126.38}, {58.08, 49.97}, {36.51, 98.49}, {102.03, 7.31}, {65.31, 80.14}, {4.19, 67.75}, {20.09, 1.8}, {29.29, 2.23}, {40.54, 82.6}, {17.37, 15.45}, {73.17, 125.25}, {92, 42.56}, {115.64, 5.97}, {66.48, 77.04}, {105.5, 1.5}, {69.76, 14.12}, {85.54, 77.97}, {25.04, 83.81}, {97.48, 70.65}, {143.78, 26.99}, {132.09, 101.29}, {85.53, 104.83}, {54.47, 146.07}, {7.73, 103.22}, {43.05, 101.76}, {0.53, 22.24}, {46.3, 53.66}, {54, 4.44}, {103.7, 2.85}, {30.16, 138.98}, {131.55, 111.33}, {5.57, 115.76}, {131.78, 132.66}, {23.57, 17.11}, {128.78, 106.14}, {98.19, 107.37}, {131.33, 55.91}, {137.2, 137.62}, {16.33, 61.97}, {110.31, 85.88}, {55.89, 98.73}, {88.77, 123.04}, {143.58, 12}, {22.54, 89.6}, {49.15, 139.47}, {54.8, 127.3}, {149.55, 95.46}, {21.07, 12.38}, {0, 149.26}, {50.35, 48.57}, {61.13, 51.14}, {95.93, 113.6}, {33.54, 34.18}, {3.57, 125.85}, {37.29, 10.15}, {65.63, 61.02}, {59.17, 66.43}, {144.19, 29.67}, {127.47, 2.69}, {103.95, 33.03}, {44.73, 141.03}, {117.48, 93.85}, {106.58, 134.59}, {54.06, 89.3}, {98.24, 1.32}, {59.73, 46.03}, {68.97, 29.2}, {139.5, 2.31}, {134.8, 52.03}, {89, 95.2}, {75.33, 112.58}, {20.03, 136.76}, {59.5, 39.34}, {87.8, 48.79}, {18.32, 125.74}}, {{135.49, 145.42}, {112.49, 27.71}, {113.1, 108.79}, {59.83, 99.7}, {20.4, 77.23}, {36.5, 29.71}, {82.29, 53.18}, {77.46, 2.99}, {72.3, 106.21}, {7.76, 51.24}, {82.44, 19.58}, {126.96, 67.71}, {140.64, 65.6}, {15.98, 87.51}, {49.4, 95.03}, {45.51, 128.01}, {72.05, 100.8}, {4.18, 42.74}, {46.49, 104.13}, {33.2, 40.25}, {20.12, 67.83}, {147.88, 141.17}, {140.08, 115.5}, {3.24, 91.95}, {112.57, 95.11}, {6.9, 36.66}, {124.1, 5.93}, {55.53, 45.65}, {59.6, 137.42}, {14.03, 43.52}, {73.07, 61.41}, {49.1, 32}, {77.99, 81.27}, {51.71, 27.87}, {144.14, 16.25}, {66.41, 145.17}, {53.48, 68.42}, {93.15, 59.74}, {14.45, 13.73}, {72.2, 149.11}, {92.39, 100.54}, {113.05, 119.29}, {122.53, 141.89}, {1.66, 93.56}, {93.04, 98.6}, {58.98, 5.92}, {67.2, 11.16}, {65.8, 88.67}, {143.52, 19.39}, {36.98, 149.01}, {58.11, 16.16}, {118.91, 86.71}, {117.28, 16.71}, {16.61, 120.45}, {31.2, 102.4}, {121.58, 94.53}, {146.27, 53.5}, {137.11, 5.63}, {45.94, 25.33}, {67.27, 28.27}, {87.95, 145.31}, {124.42, 115.16}, {50.06, 85.15}, {69.24, 76.01}, {120.97, 36.61}, {121.03, 93.24}, {1.08, 49.36}, {72.81, 20.15}, {87.1, 132.16}, {69.05, 59.73}, {127.81, 127.2}, {107.91, 61.69}, {78.65, 108.27}, {45.36, 67.28}, {121.25, 103.84}, {1.67, 149.49}, {4.76, 60.46}, {65.76, 136.66}, {74.67, 105.2}, {73.84, 119.91}, {2.1, 82.58}, {82.11, 87.44}, {145.18, 7.18}, {69.34, 18.25}, {7.67, 136.89}, {98.18, 80.14}, {68.25, 129.18}, {30.28, 30.29}, {80.34, 17.27}, {62.4, 37.54}, {15.4, 83.96}, {109.35, 130.83}, {80.48, 118.65}, {20.11, 2.52}, {80.64, 78.76}, {72.37, 126.75}, {52.45, 77.1}, {54.15, 81.92}, {97.1, 97.47}, {130.37, 112.99}}, {{34.47, 147.15}, {66.93, 45.79}, {121.29, 51.64}, {39.79, 35.01}, {55.25, 102.89}, {99.58, 140.33}, {18, 100.89}, {76.7, 2.17}, {58.46, 60.38}, {145.43, 3.38}, {59.36, 133.21}, {127.19, 64.96}, {12.47, 87.88}, {83.59, 9.87}, {100.35, 52.35}, {89.12, 56.84}, {109.89, 137.84}, {137.34, 26.67}, {105.19, 26.15}, {20.45, 59.76}, {116.61, 108.25}, {28.19, 16.31}, {129.03, 26.98}, {17.98, 26.39}, {40.04, 45.03}, {106.79, 137.65}, {73.91, 91.07}, {149.24, 143.65}, {71.21, 94.54}, {21.06, 24.5}, {67.03, 108.32}, {10.32, 71.41}, {16.54, 69.77}, {122.17, 139}, {72.89, 134.65}, {47.47, 86.22}, {48.79, 43.17}, {65.16, 140.48}, {77.62, 60.29}, {90.39, 27.08}, {59.03, 26.57}, {107.61, 7.52}, {64.66, 36.21}, {145.57, 119.96}, {69.55, 19.3}, {14.94, 105.55}, {22.21, 93.12}, {22.93, 14.27}, {72.32, 74.44}, {147.91, 21.98}, {91.52, 98.56}, {135.7, 143.03}, {74.39, 129.14}, {7.99, 67.66}, {18.35, 19.21}, {110.43, 77.52}, {140.86, 47.19}, {141.5, 84.1}, {136.23, 146.69}, {146.05, 149.43}, {123.11, 24.31}, {29.92, 148.16}, {136.99, 119.71}, {56.39, 13.7}, {58.36, 148.91}, {128.57, 48.33}, {33.28, 31.77}, {44.1, 126.83}, {101.76, 84.89}, {20.91, 4.34}, {147.11, 69.04}, {10.26, 48.91}, {57.9, 105.22}, {93.64, 68.9}, {148.84, 68.13}, {64.41, 121.27}, {11.79, 107.84}, {145.37, 48.7}, {25.61, 25.81}, {141.65, 108.02}, {77.32, 73.32}, {128.54, 47.59}, {148.47, 73.77}, {54.62, 40.1}, {138.19, 31.5}, {133.77, 9.41}, {38.36, 67.2}, {122.99, 11.12}, {47.59, 99.56}, {43.9, 111.52}, {0.48, 50.14}, {140.44, 99.18}, {1.5, 1.01}, {12.95, 48.21}, {107.31, 145.01}, {132.45, 49.61}, {60.57, 85.1}, {89.28, 58.05}, {21.73, 21.37}, {146.69, 140.96}}, {{45.03, 24.21}, {86.75, 35.18}, {93, 64.89}, {2.62, 70.54}, {132.63, 36.43}, {148.89, 52.5}, {117.35, 144.15}, {13.64, 124.94}, {9.63, 77.58}, {106.52, 50.92}, {123.16, 17.14}, {114.2, 130.6}, {124.5, 133.68}, {37.41, 118.47}, {91.96, 47.87}, {125.01, 132.34}, {81.28, 55.71}, {74.48, 128.75}, {122.53, 118.71}, {143.29, 72.93}, {137.19, 34.71}, {138.2, 107.64}, {60.79, 26.3}, {144.1, 80.8}, {111.13, 33.42}, {31.38, 33.82}, {110.44, 38.28}, {133.93, 35.75}, {109.8, 140}, {105.96, 109.47}, {23.7, 10.87}, {15.43, 27.33}, {128.52, 52.61}, {18.24, 144}, {23.8, 57.78}, {118.57, 34.23}, {44.98, 70.14}, {28.45, 66.62}, {35.09, 66.1}, {18.49, 142.18}, {92.9, 50.55}, {107.43, 35.55}, {13.85, 15.34}, {12.75, 73.21}, {61.51, 12.26}, {2.51, 94.59}, {77.83, 117.14}, {14.37, 128.79}, {26.98, 41}, {81.37, 42.85}, {8.78, 32.84}, {3.42, 49.15}, {59.6, 130.67}, {101.4, 7.07}, {35.66, 57.28}, {5.62, 13.63}, {119.69, 78.9}, {26.23, 122.78}, {23.13, 107.61}, {70.1, 98.4}, {119.78, 60}, {23.94, 95.07}, {76.56, 65.51}, {117.86, 84.31}, {95.44, 57.39}, {137.76, 75.84}, {92.52, 41.08}, {105.14, 64.54}, {0.19, 30.28}, {66.71, 123.7}, {116.63, 129.47}, {90.24, 145.43}, {68.54, 110.08}, {149.81, 55.52}, {62.88, 81.63}, {133.59, 10.19}, {103.79, 3.56}, {94.61, 34.87}, {33.53, 38.99}, {57.88, 92.44}, {148.8, 24.66}, {32.83, 2.74}, {2.36, 139.09}, {35.67, 18.34}, {29.39, 46.84}, {96.51, 145.49}, {142.66, 124.67}, {51.1, 45.89}, {13.27, 27.73}, {5.81, 42.95}, {79.36, 128.74}, {4.35, 132.31}, {119.72, 30.28}, {47.98, 40.7}, {45.87, 62.97}, {140.89, 26.46}, {140.9, 131.11}, {16.12, 74.63}, {121.12, 37.81}, {69.51, 142.3}}, {{33.21, 32.13}, {101.28, 19.5}, {86, 139.59}, {147.54, 149.92}, {101.94, 133.3}, {136.32, 135.59}, {37.77, 127.79}, {36.32, 132.72}, {113.83, 40.81}, {121.53, 76.22}, {85.21, 129.56}, {48.18, 55.91}, {19, 51.96}, {141.2, 27.04}, {35.99, 121.29}, {71.44, 41.75}, {108.7, 84.16}, {45.99, 20.8}, {146.18, 92.88}, {144.52, 131.28}, {128, 126.62}, {148.4, 41.49}, {113.85, 65.83}, {145.15, 56.59}, {48.78, 9.59}, {85.61, 83.82}, {30.48, 90.54}, {44.4, 63.81}, {87.7, 120.28}, {102.51, 25.44}, {8.69, 75.95}, {24.69, 27.1}, {81.72, 9.23}, {21.25, 62.19}, {88.5, 14.31}, {112.43, 28.46}, {58.69, 66.7}, {63.94, 146.1}, {63.92, 10.64}, {45.24, 143.61}, {45.19, 136.2}, {15.62, 16.61}, {37.25, 44.34}, {59.49, 120.9}, {123.22, 12.6}, {50.93, 99.28}, {101.22, 63.81}, {63.66, 114.68}, {110.23, 127.22}, {116.39, 35.17}, {87.93, 93.64}, {41.78, 19.56}, {67.71, 83.41}, {63.55, 58.36}, {84.89, 97.28}, {121.2, 149.02}, {103.12, 146.16}, {83.47, 89.46}, {78.51, 126.72}, {108.11, 15.23}, {117.36, 126.05}, {96.07, 56.33}, {28.66, 115.53}, {14.12, 29.25}, {90.4, 91.53}, {42.57, 117.33}, {82.95, 81.39}, {22.51, 12.58}, {126.94, 141.6}, {61.38, 99.29}, {148.75, 45.98}, {92.92, 56.33}, {95.62, 82.76}, {32.36, 6.46}, {38.73, 94.64}, {74.84, 71.92}, {41.19, 88.42}, {93.28, 138.59}, {52.62, 141.45}, {7.16, 21.27}, {121.72, 87.48}, {140, 93.7}, {63.98, 112.92}, {45.45, 39.28}, {73, 21.43}, {71.92, 35.63}, {47.83, 42.72}, {48.01, 90.09}, {112.21, 144.86}, {45.13, 107.69}, {75.31, 3.57}, {108.19, 44.86}, {68.56, 48.16}, {92.54, 97.54}, {48.14, 76.15}, {115.37, 79.66}, {22.97, 38.77}, {107.6, 24.12}, {101.91, 132.31}, {141.74, 74.63}}, {{98.77, 3.24}, {97.84, 78.33}, {117.5, 145.27}, {149.99, 124.29}, {46.09, 21.44}, {53.64, 24.51}, {128.12, 19.79}, {91.34, 74.07}, {135.83, 76}, {142.59, 133.1}, {7.53, 108.91}, {43.21, 82.25}, {52.53, 39.29}, {110.96, 55.53}, {6.61, 48.58}, {24.91, 129.63}, {129.2, 7.68}, {16.87, 65.26}, {4.72, 52.19}, {56.41, 82.04}, {129.51, 30.92}, {134.45, 113.7}, {94.74, 19.62}, {61.76, 103.17}, {8.26, 84.13}, {83.44, 125.71}, {59.79, 110.35}, {131.29, 72.88}, {98.48, 59.78}, {106.17, 127.23}, {128.38, 45.08}, {139.29, 49.26}, {99.68, 68.06}, {101.99, 122.55}, {48.29, 32.76}, {34.88, 128.96}, {128.19, 18.64}, {61.37, 124.16}, {103.18, 44.91}, {56.83, 83.98}, {111.2, 97.48}, {99.61, 56.51}, {110.18, 109.77}, {99.46, 36.61}, {64.78, 72.28}, {0.63, 2.35}, {65.06, 63.08}, {89.68, 6.79}, {91.19, 23.23}, {46.77, 43.92}, {39.53, 19.78}, {68.53, 92.95}, {82.71, 84.75}, {52.04, 119.62}, {30.32, 138.19}, {85.31, 90.35}, {144.68, 124.68}, {76.1, 2.34}, {72.7, 117.78}, {112.65, 90.45}, {82.79, 90.75}, {3.27, 76.87}, {103.63, 90.37}, {7.66, 45.34}, {30.83, 136.75}, {111.84, 45.55}, {91.56, 123.99}, {136.64, 70.93}, {78.86, 56.33}, {49.24, 130.47}, {1.94, 113.3}, {117.67, 106.65}, {69.1, 66.82}, {118.05, 13.14}, {46.52, 62.41}, {135.85, 125.91}, {128.06, 97.6}, {79.6, 9.12}, {104.03, 34.3}, {10.85, 92}, {135.85, 144.67}, {4.84, 139.88}, {102, 56.86}, {1, 4.95}, {131.99, 11.02}, {44.28, 71.83}, {139.17, 21.36}, {128.02, 94.89}, {65.78, 147.25}, {57.89, 67.22}, {5.95, 106.05}, {123.19, 36.91}, {57.21, 27.74}, {35.14, 98.62}, {57.06, 62.83}, {96.26, 114.19}, {138.01, 143.3}, {76.44, 73.75}, {132.58, 114.62}, {131.31, 17.36}}, {{57.2, 83.08}, {110.24, 119.47}, {5.08, 34.11}, {43.97, 10}, {33.14, 118.96}, {139.99, 6.45}, {25.33, 49.62}, {5.39, 137.54}, {138.21, 42.86}, {115.36, 100.1}, {146.62, 31.73}, {44.93, 116.35}, {49.88, 95.71}, {68.38, 98.21}, {148.91, 84.13}, {105.6, 149.57}, {40.92, 87.43}, {47.75, 27.68}, {4.13, 54.15}, {116.36, 48.91}, {83.47, 143.69}, {48.32, 141.34}, {89.28, 51.43}, {8.79, 109.06}, {40.54, 8.07}, {25.37, 2.03}, {144.21, 74.47}, {110.34, 19.49}, {94.16, 77.59}, {119.38, 135.5}, {101.83, 21.86}, {110.96, 146.85}, {105.24, 79.22}, {3.63, 91.81}, {30.12, 32.51}, {122.02, 54.95}, {115.36, 84.54}, {72.08, 119.8}, {30.31, 8.3}, {20.95, 67.14}, {31.12, 21.36}, {99.37, 120.09}, {6.85, 129.19}, {143.81, 46.1}, {149.08, 116.09}, {44.39, 128.67}, {11.19, 12.66}, {49.99, 145.2}, {145.15, 75.77}, {49.22, 121.42}, {39.8, 108.81}, {18.85, 50.3}, {13.25, 124.36}, {19.94, 139.45}, {26.02, 132.33}, {34.8, 84.56}, {28.34, 115.17}, {38.81, 87.81}, {84.58, 128.05}, {99.92, 41.6}, {86.51, 12.01}, {85.42, 139.44}, {46.35, 75.55}, {18.72, 74.43}, {73.22, 120.13}, {34.41, 47.81}, {107.79, 95.18}, {71.6, 117.29}, {99.55, 90.82}, {43.64, 130.32}, {143.13, 91.95}, {93.28, 125.04}, {98.55, 77.11}, {88.81, 116.1}, {99.04, 125.51}, {76.16, 17.61}, {88.64, 104.05}, {21.22, 51.13}, {49.43, 136.36}, {1.18, 79.94}, {71.88, 87.43}, {126.39, 145.02}, {53.43, 100.7}, {12.19, 32.12}, {133.17, 49.32}, {77.82, 90.97}, {78.59, 120.73}, {55.9, 98.19}, {140.1, 9.11}, {80.15, 49.78}}, {{85.42, 60.45}, {82.7, 55.62}, {80.08, 39.83}, {60.99, 7.09}, {132.13, 35.5}, {129.51, 46.65}, {35.02, 16.17}, {149.53, 52.99}, {79.16, 125.84}, {68.69, 24.42}, {35.03, 115.15}, {93.29, 22.57}, {33.82, 2.64}, {143.52, 1.54}, {125.26, 60.27}, {137.55, 42.59}, {79.52, 23.86}, {24.22, 41.29}, {137.86, 55.49}, {18.79, 141.98}, {88.9, 86.03}, {96.86, 24.43}, {38.27, 85.85}, {101.98, 85.87}, {108.17, 40.31}, {84.76, 124.61}, {65.63, 2.14}, {13.83, 18.32}, {47.22, 105.11}, {127.91, 128.71}, {61.42, 97.33}, {65.53, 130.3}, {123.8, 37.04}, {99.15, 88.87}, {71.1, 116.81}, {65.58, 87.86}, {14.33, 140.4}, {26.59, 19.36}, {125.62, 19.38}, {147.54, 145.26}, {85.26, 47.12}, {125.95, 136.21}, {11.77, 53.41}, {141.58, 30.54}, {110.2, 71.97}, {47.27, 24.51}, {144.62, 103.32}, {129.37, 58.04}, {110.57, 44.05}, {9.07, 49.43}, {142.58, 15.39}, {73.43, 136.97}, {22.81, 79.65}, {114.57, 6.05}, {51.17, 34.12}, {106.24, 101.84}, {14.59, 102.99}, {22.16, 97.4}, {118.51, 28.46}, {79.22, 18.93}, {136.15, 30.49}, {13.61, 136.49}, {122.31, 27.96}, {111.23, 106.96}, {24.44, 52.5}, {112.38, 24.31}, {93.59, 59.35}, {62.65, 128.01}, {133.87, 74.37}, {39.48, 51.2}, {4.24, 138.94}, {77.75, 79.03}, {49.89, 12.86}, {108.45, 41.43}, {122.57, 63.32}, {82.61, 105.78}, {21.04, 142.02}, {120.31, 32.2}, {96.86, 36.3}, {36.43, 73.67}, {106.48, 138.44}, {73.17, 47.49}, {51.25, 12.73}, {78.63, 104.16}, {116.8, 103.48}, {109.79, 9.57}, {18.14, 27.05}, {21.89, 83.07}, {49.05, 95.27}, {86.14, 87.15}}, {{143.08, 88.97}, {44.49, 74.95}, {60.29, 40.26}, {28.66, 93.98}, {89.85, 66.62}, {25.18, 38.53}, {5.79, 112.03}, {83.83, 57.23}, {105.59, 132.21}, {61.74, 90.34}, {126.75, 107.02}, {94.36, 32.79}, {29.15, 47.5}, {30.13, 88.5}, {74.92, 79.48}, {90.83, 29.81}, {58.47, 132.08}, {77.06, 101.78}, {23.06, 82.67}, {141.14, 39.1}, {94.7, 123.58}, {86.17, 149.46}, {73.04, 126.56}, {46.63, 86.37}, {148.98, 67.34}, {86.56, 85.08}, {130.12, 91.13}, {12.8, 17.24}, {91.04, 46.13}, {120.73, 49.62}, {29.79, 93.77}, {132.69, 111.47}, {106.31, 97.53}, {56.33, 37.39}, {125.2, 61.94}, {25.97, 144.24}, {118.71, 99.58}, {83.2, 39.48}, {130.18, 24.01}, {105.17, 63.92}, {126.13, 35.8}, {63.56, 79.4}, {35.94, 38.68}, {106.7, 124.63}, {66.49, 51.34}, {114.25, 48.71}, {18.91, 6.5}, {12.17, 0.1}, {26.54, 115.25}, {89.01, 4.94}, {63.92, 30.34}, {99.7, 125.46}, {11.87, 11.21}, {15.65, 85.89}, {29.71, 148.65}, {61.6, 21.1}, {57.75, 31.09}, {146.94, 33.23}, {65.34, 84.74}, {94.14, 4.94}, {54.23, 118.75}, {130.77, 13.57}, {6.27, 52.22}, {105.74, 1.72}, {38.39, 90.39}, {81.83, 144.17}, {5.3, 38.9}, {76.06, 48.96}, {70.12, 8.95}, {69.38, 25.16}, {3.21, 96.84}, {92.27, 5.78}, {144.6, 69.72}, {84.87, 136.35}, {15.81, 55.26}, {41.94, 120.3}, {12.95, 92.58}, {134.19, 122.12}, {34.43, 73.12}, {137.06, 16.28}, {115.93, 39.26}, {112.26, 77.26}, {127.33, 79.81}, {102.96, 93.92}, {130.61, 140.09}, {13.77, 111.24}, {105.22, 92.4}, {10.52, 78.15}, {51.8, 12.92}, {21.51, 110.52}}, {{149.73, 96.77}, {54.17, 126.26}, {74.65, 12.61}, {77.39, 108.49}, {0.07, 77.19}, {24.3, 103.76}, {78.25, 135.81}, {33.27, 136.59}, {134.97, 61.93}, {32.25, 148.29}, {88.22, 113.48}, {75.53, 108.79}, {94.68, 12.27}, {124.24, 14.31}, {92.77, 114.06}, {105, 112.72}, {116.01, 89.3}, {6.98, 140.77}, {45.49, 72.94}, {21.27, 36.52}, {100.59, 145.77}, {46.7, 31.51}, {122.24, 20.84}, {100.2, 100.15}, {102.19, 141.77}, {36.95, 18.95}, {106.64, 6}, {1.18, 1.17}, {137.23, 16.17}, {87.54, 95.68}, {35.87, 45.75}, {49.13, 81.22}, {91.57, 107.16}, {105.82, 51.42}, {59.62, 80.08}, {134.4, 23.05}, {111.7, 74.06}, {2.01, 37.19}, {16.2, 29.62}, {103.17, 121.35}, {67.52, 8.94}, {32.35, 146.1}, {58.31, 43.63}, {68.51, 88.47}, {4.51, 61.1}, {55.49, 94.47}, {12.34, 67.48}, {42.23, 130.88}, {17.71, 9.32}, {39.59, 22.56}, {103.45, 47.02}, {97.23, 28.89}, {68.19, 24.94}, {98.86, 21.07}, {73.92, 75}, {149.46, 104.01}, {43.65, 27.44}, {77.56, 144.74}, {29.17, 57.75}, {36.95, 28.66}, {18.64, 114.91}, {7.91, 49.4}, {37.56, 90.59}, {48.97, 30.97}, {103.09, 105.63}, {110.22, 90.85}, {118.99, 135.19}, {17.2, 23.92}, {1.48, 140.53}, {107.76, 28.76}, {18.33, 87.97}, {51.93, 29.03}, {130.08, 143.74}, {17.66, 63.03}, {66.3, 125.81}, {141.86, 87.25}, {26.69, 64.25}, {101.58, 81.85}, {23.99, 54.21}, {81.14, 102.6}, {100.56, 48.72}, {100.53, 138.53}, {107.75, 95.81}, {95.55, 11.62}, {0.48, 27.45}, {44.52, 54.7}, {66.31, 84}, {122.63, 45.47}, {89.19, 73.7}, {32.01, 89.59}}};

vector<uchar> statusRemove {0, 0, 0, 0, 0, '1', '1', '1', '1', '1', 0, '1', '1', '1', '1', '1', '1', '1', '1', '1', 0, '1', '1', '1', '1', '1', '1', '1', '1', '1', 0, '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', 0, '1', '1', '1', '1', '1', '1', '1', 0, '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'};

vector<uchar> allGoodStatus {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'};



void drawBasedOnBreaks(const vector<double>& naturalBreaks, const vector<double>& values, Mat& image, vector< vector<Point2f> > tracking){
    
    for (int p = 0; p<values.size(); ++p)
    {
        if (values[p] < naturalBreaks[0])
        {
            // Draw Blue Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[1])
        {
            // Draw Green Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,0), -1, 8);
        }
        else if (values[p] < naturalBreaks[2])
        {
            //Draw Red Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(0,0,255), -1, 8);
        }
        
        
        else if (values[p] < naturalBreaks[3])
            
        {
            //Draw Purple Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(255,0,255), -1, 8);
        }
        else
        {
            // Draw Yellow Circles
            circle( image, tracking[p].rbegin()[0], 3, Scalar(0,255,255), -1, 8);
            
        }
        
    }
    
}

void generateMask(cv::Mat& mask, cv::Mat& image, vector<Point2f>& points){
    double length = 20.0;
    int cols = image.cols;
    int rows = image.rows;
    mask = cv::Mat::ones(rows, cols, CV_8UC1)*255;
    for (int i = 0; i < points.size(); i++){
        float x = points[i].x-length/2;
        float y = points[i].y-length/2;
        if (x < 0 || y < 0 || x+length > cols || y+length > rows) {
            continue;
        }
        
        Rect ROI (x, y, length, length);
        mask(ROI).setTo(Scalar::all(0));
    }
}



void concatenateVectors (vector<Point2f>& a, vector<Point2f>& b){
    a.insert(std::end(a), std::begin(b), std::end(b));
}

void concatenateVectors (vector<uchar>& a, vector<uchar>& b){
    a.insert(std::end(a), std::begin(b), std::end(b));
}

void featuresInit (vector<Point2f>& pointsInit, const int& maxPoint, const int& numPoints, Mat& mask, const Mat& gray, const TermCriteria& termcrit, const Size& subPixWinSize){
    goodFeaturesToTrack(gray, pointsInit, maxPoint-numPoints, 0.01, 10, mask, 3, 0, 0.04);
    cornerSubPix(gray, pointsInit, subPixWinSize, Size(-1,-1), termcrit);
    
}

//------------------------- Entropy Calculation --------------------------

double euclideanDistance(const Point2f& p1,const Point2f& p2)
{
    Point2f difference;
    difference = p1-p2;
    return sqrt(pow(difference.x,2)+pow(difference.y,2));
}

// calculate the minimum radius that encloses all the points using the MiniBall Software(V3.0)
double calcMinRadius(const vector<Point2f>& points, vector<Point2f>& supportPoints, double& radius, Point2f& center)
{
    vector<int> supportPointIndices;
    int cases;
    std::vector<std::vector<double> > lp;
    if (!supportPoints.empty() && euclideanDistance(points.back(), center) <= radius) {
        return radius;
    }
    else if (!supportPoints.empty() && euclideanDistance(points.back(), center) > radius){
        supportPoints.push_back(points.back());
        lp.resize(supportPoints.size());
        for (int i = 0; i < supportPoints.size(); i++) {
            std::vector<double> p(2);
            p[0] = supportPoints[i].x;
            p[1] = supportPoints[i].y;
            lp[i] = p;
        }
        cases = 1;
    }
    else{
        // convert vector<Point2f> to vector<double>
        lp.resize(points.size());
        for (int i=0; i<points.size(); ++i) {
            std::vector<double> p(2);
            p[0] = points[i].x;
            p[1] = points[i].y;
            lp[i] = p;
        }
        cases = 0;
    }
    // define the types of iterators through the points and their coordinates
    typedef std::vector<std::vector<double> >::const_iterator PointIterator;
    typedef std::vector<double>::const_iterator CoordIterator;
    
    // create an instance of Miniball
    typedef Miniball::
    Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> >
    MB;
    MB mb (2, lp.begin(), lp.end());
    
    // return radius
    // support points on the boundary determine the smallest enclosing ball
//    std::cout << "Number of support points:\n  ";
//    std::cout << mb.nr_support_points() << std::endl;
    supportPointIndices.resize(mb.nr_support_points());
    
//    std::cout << "Support point indices (numbers refer to the input order):\n  ";
    MB::SupportPointIterator it = mb.support_points_begin();
    PointIterator first = lp.begin();
    for (int i = 0; it != mb.support_points_end(); ++it, i++) {
        //std::cout << std::distance(first, *it) << " "; // 0 = first point
        supportPointIndices[i] = std::distance(first, *it);
    }
    //std::cout << std::endl;
    if (cases == 0) {
        supportPoints.resize(supportPointIndices.size());
        for (int i = 0; i < supportPoints.size(); i++) {
            supportPoints[i] = points[supportPointIndices[i]];
        }
    }
    
    if (cases == 1) {
        vector<Point2f> tmp(supportPointIndices.size());
        for (int i = 0; i<supportPointIndices.size(); i++) {
            tmp[i] = supportPoints[supportPointIndices[i]];
        }
        supportPoints = tmp;
    }
    
    //std::cout << "Center:\n  ";
    const double* center1 = mb.center();
    center.x = center1[0];
    center.y = center1[1];
//    for(int i=0; i<2; ++i, ++center1)
//        std::cout << *center1 << " ";
//    std::cout << std::endl;
    radius = sqrt(mb.squared_radius());
    
    return sqrt(mb.squared_radius());
}


void calcEntropyUsingMiniballRadiusIterative (double& entropy, double& length, vector<Point2f>& points, float &maxRadius, vector<Point2f>& supportPoints, double& radius, Point2f& center)
{
    if (points.size() < 3){
        entropy = 0;
    }
    else {
        double radius;
        radius = calcMinRadius(points, supportPoints, radius, center);
        if (radius > maxRadius)
            maxRadius = radius;
        entropy = log(length/(2*radius))/(log(points.size()-1))*radius/maxRadius;
    }
}


void calcLengthIterative(double& length, Point2f& p1, Point2f& p2)
{
    Point2f difference;
    difference = p1-p2;
    length += sqrt(pow(difference.x,2)+pow(difference.y,2));
}

void entropyListUpdate (vector<double>& entropy, vector<double>& lengths, vector< vector<Point2f> >& allTrackers, float& maxRadius, vector< vector<Point2f> >& supportPoints,  vector<double>& radius, vector<Point2f>& center)
{
    size_t n = allTrackers.size();
    for (int i = 0 ; i < n; i++) {
        if (allTrackers[i].size() < 2){
            continue;
        }
        calcLengthIterative(lengths[i], allTrackers[i].rbegin()[0], allTrackers[i].rbegin()[1]);
        calcEntropyUsingMiniballRadiusIterative(entropy[i], lengths[i], allTrackers[i], maxRadius, supportPoints[i], radius[i], center[i]);
    }
}



//------------------------ Entropy Calculation End --------------------------



vector<vector<Point2f>> initTracking (vector<Point2f>& initPoints){
    vector<vector<Point2f>> initTracking(initPoints.size());
    
    for (int i = 0; i < initPoints.size(); i++){
        initTracking[i].push_back (initPoints[i]);
    }
    return initTracking;
}

void writeVector(ofstream& writeFile, vector<Point2f>& vector){
    writeFile << "{";
    for (int i = 0 ; i < vector.size(); i++) {
        writeFile << "{"<< vector[i].x << ", " << vector[i].y << "}";
        if (i == vector.size() - 1) {
            continue;
        }
        writeFile << ", ";
    }
    writeFile << "}";
}

void writeVector(ofstream& writeFile, vector<double>& vector){
    writeFile << "{";
    for (int i = 0 ; i < vector.size(); i++) {
        writeFile << vector[i];
        if (i == vector.size() - 1) {
            continue;
        }
        writeFile << ", ";
    }
    writeFile << "}";
}

void writeVector(ofstream& writeFile, vector<int>& vector){
    writeFile << "{";
    for (int i = 0 ; i < vector.size(); i++) {
        writeFile << vector[i];
        if (i == vector.size() - 1) {
            continue;
        }
        writeFile << ", ";
    }
    writeFile << "}";
}

void writeVectorVector (ofstream& writeFile, vector<vector<Point2f>>& vectorVector){
    writeFile << "{";
    for (int i = 0; i < vectorVector.size(); i++) {
        writeVector(writeFile, vectorVector[i]);
        if (i == vectorVector.size() - 1) {
            continue;
        }
        writeFile << ", ";
    }
    writeFile << "}";
}

void writeVectorVector (ofstream& writeFile, vector<vector<int>>& vectorVector){
    writeFile << "{";
    for (int i = 0; i < vectorVector.size(); i++) {
        writeVector(writeFile, vectorVector[i]);
        if (i == vectorVector.size() - 1) {
            continue;
        }
        writeFile << ", ";
    }
    writeFile << "}";
}



int main(int argc, const char* argv[])
{
    char* movie;
    movie = "/Users/student/Desktop//GP058145.m4v";
    //GP058145.m4v";
    //OpenCV/RiverSegmentation/RiverSegmentation/MovieBoat.mp4";
    VideoCapture cap;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    
    if (testing) {
        report.open("reportDebug1.txt", ios::out);
    }
    if (profiling) {
        report.open("reportProfiling1.txt", ios::out);
    }
    
    const int MAX_COUNT = 400;
    bool needToInit = true;
    bool addRemovePt = false;
    
    
    cap.open(movie);
    
    if( argc == 2 )
        cap.open(argv[1]);
    
    else
        cap.open(movie);
    
    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    
    namedWindow("Entropy Based River Segmentation", 1 );
    namedWindow("Mask",1);
    
    VideoWriter outputVideo;
    bool writeMovie = false; //write the output video to the place defined by filename
    if (writeMovie)
    {
        char* filename;
        filename = "/Users/student/Desktop/OpenCV/RiverSegmentation/RiverSegmentation/MovieBoatOutputDone.mp4";
        
        Size S = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                      (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        outputVideo.open(filename, CV_FOURCC('8','B','P','S'), 30, S);
        if (!outputVideo.isOpened())
        {
            cout  << "Could not open the output video for write: " << filename << endl;
            return -1;
        }
        
    }
    
    Mat gray, prevGray, image, mask;
    
    vector< vector<Point2f> > points1;
    vector< vector<Point2f> > points0;
    
    vector< vector< vector<Point2f> > > tracking;// (1, vector< vector<Point2f> > (MAX_COUNT, vector<Point2f>()));
    

    vector<double> naturalBreaks;
    
    //----- Entropy Values ------
    vector< vector<double> > lengths;//(1, vector<double> (MAX_COUNT));
    vector< vector<double> > entropy;//(1, vector<double> (MAX_COUNT));
    vector< vector< vector<Point2f> > > supportPoints;
    vector< vector<double> > radius;
    vector< vector<Point2f> > center;
    vector<float> maxRadius;
    
    //----- End Entropy Values ----
    
    //Variables for Dissimilarity

    vector<int> numberPerTime;
    //vector< vector<int> >removedIndex;
    

    vector<int> count;
    int c = 0;
    int numPoints = 0;
    
    for(;;)
    {
        if(testing){
            if( c >= storedPoints.size() )
                break;
        }
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        
        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        
        
        if( needToInit )
        {
            points1.push_back({});
            //vector<Point2f> pointsInit;
            cout << "Generating NEW POINTS";
            //generateMask(mask, image, points[1]);
            // Using goodFeaturesToTrack function to automatically find 500 features to track
            goodFeaturesToTrack(gray, points1.back(), MAX_COUNT-numPoints, 0.01, 10, mask, 3, 0, 0.04);
            cornerSubPix(gray, points1.back(), subPixWinSize, Size(-1,-1), termcrit);
            
            addRemovePt = false;
            
            if (testing) {
                points1.back() = storedPoints[0];
            }
            //concatenateVectors(points[1], pointsInit);
            
            
            //neighbourIndexList.resize(points[1].size());
            
            //cout << pointsInit.size() << endl;
            //cout << numPoints << endl;
            
            numPoints = points1.back().size();
            
            //cout << numPoints << endl;
            
            tracking.push_back(initTracking(points1.back()));
            
            //----- Initializing Entropy Values --------
            
            entropy.push_back(vector<double> (points1.back().size()));
            
            lengths.push_back(vector<double> (points1.back().size()));
            
            supportPoints.push_back(vector<vector<Point2f>> (points1.back().size()));
            
            center.push_back(vector<Point2f> (points1.back().size()));
            
            radius.push_back(vector<double> (points1.back().size()));
            
            maxRadius.push_back(0.0);
            
            
            //----- End Initializing Entropy Values -------
            
            count.push_back(0);
            //cout << "initiating points" << endl;
            needToInit = false;
            //std::swap(points[1], points[0]);
            points0.push_back({});
            
            
        }
        
        for (int i = 0; i < tracking.size(); i++) {
            if( !points0[i].empty() )
            {
                vector<uchar> status;
                vector<float> err;
                vector<int> removedIndex;

                if(prevGray.empty()){
                    gray.copyTo(prevGray);
                }
                
                calcOpticalFlowPyrLK(prevGray, gray, points0[i], points1[i], status, err, winSize, 3, termcrit, 0, 0.001);
                
                if (testing) {
                    status = allGoodStatus;
                    
                    if (c == 5){
                        status = statusRemove;
                    }
                    
                    
                    points1[i] = storedPoints[c];
                    //cout << c;
                    if (c > 5 && i == 1) {
                        points1[i] = storedPoints[c-6];
                        //concatenateVectors(status, allGoodStatus);
                    }
                }
                
                
                size_t l, k;
                for( l = k = 0; l < points1[i].size(); l++ )
                {
                    
                    if( !status[l] )
                    {
                        removedIndex.push_back(l);
                        continue;
                    }
                    points0[i][k] = points0[i][l];
                    points1[i][k] = points1[i][l];
                    tracking[i][k] = tracking[i][l];
                    tracking[i][k].push_back(points1[i][l]);
                    
                    
                    //---- Entropy Values ---
                    lengths[i][k] = lengths[i][l];
                    radius[i][k] = lengths[i][l];
                    center[i][k] = center[i][l];
                    supportPoints[i][k] = supportPoints[i][l];
                    
                    //---- End Entropy Values
                    
                    k++;
                }
                
                points0[i].resize(k);
                points1[i].resize(k);
                tracking[i].resize(k);
                
                //---- Entropy Values ---
                
                entropy[i].resize(k);
                lengths[i].resize(k);
                center[i].resize(k);
                radius[i].resize(k);
                supportPoints[i].resize(k);
                
                //---- End Entropy Values
                
                start = std::chrono::high_resolution_clock::now();
                entropyListUpdate(entropy[i], lengths[i], tracking[i], maxRadius[i], supportPoints[i],radius[i],center[i]);
                ending = std::chrono::high_resolution_clock::now();
                timeSpan = duration_cast<duration<double>>(ending - start);
                if (profiling) {
                    report << "Running time of entropyListUpdate :" << timeSpan.count() << endl;
                }

                count[i]++;
                
                
            }
            
            
            if (count[i] > 3)
            {
                
                naturalBreaks = JenksNaturalBreak(entropy[i], 4);
                if (testing) {
                    continue;
                }
                drawBasedOnBreaks(naturalBreaks, entropy[i], image, tracking[i]);
                
            }
            
            
            
            if (testing) {
                
                report << c << endl;
                
                report << "points[0]" << endl;
                writeVector(report, points0[0]);
                report << endl;
                
                report << "points[1]" << endl;
                writeVector(report, points1[0]);
                report << endl;
                
                report << "t = " << i << endl;
                
                report << "lengths" << endl;
                writeVector(report, lengths[i]);
                report << endl;
                
                report << "entropy" << endl;
                writeVector(report, entropy[i]);
                report << endl;
                
                
                report << "naturalBreaks" << endl;
                writeVector (report, naturalBreaks);
                report << endl;
                
                //                    report << "removedIndex" << endl;
                //                    writeVector(report, removedIndex);
                //                    report << endl;
                
                report << "tracking" << endl;
                writeVectorVector(report, tracking[i]);
                report << endl;
                
                report << "SupportPoints" << endl;
                writeVectorVector(report, supportPoints[i]);
                report << endl;
                
                report << "Radius" << endl;
                writeVector(report, radius[i]);
                report << endl;
                
                report << "Center" << endl;
                writeVector(report, center[i]);
                report << endl;
                
            }
            
            
            std::swap(points1[i], points0[i]);
        }
        numPoints = 0;
        vector<Point2f> allPoints;
        //cout << numPoints << ": NUMPOINTS";
        for (int i = 0; i < points0.size(); i++) {
            numPoints += points0[i].size();
            concatenateVectors(allPoints, points0[i]);
        }
        if (numPoints < MAX_COUNT*0.9) {
            needToInit = true;
            cout << "Init Points" << endl;
            generateMask(mask, image, allPoints);
        }
        if (testing) {
            needToInit = false;
            if (c == 5){
                needToInit = true;
            }
        }
        
        c++;
        
        cv::swap(prevGray, gray);
        
        imshow("Entropy Based River Segmentation", image);
        if(!mask.empty()){
            imshow("Mask", mask);
        }
        
        
        char d = (char)waitKey(30);
        if( d == 27 )
            break;
        
        if (writeMovie)
        {
            outputVideo.write(image);
        }
        
        
        
        
    }
    
    report.close();
    
    return 0;
}