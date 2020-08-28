const unsigned int knockSampleRate = 22050;
const unsigned int knockSampleCount = 1241;
const signed char knockSamples[] = {
0, -1, -3, -4, -5, -5, -4, -2, 2, 7, 11, 15, 15, 12, 10, 7, 
6, 8, 12, 18, 23, 26, 25, 22, 17, 14, 11, 9, 10, 14, 17, 21, 
23, 24, 24, 25, 26, 28, 32, 37, 45, 48, 50, 50, 48, 45, 41, 40, 
40, 43, 46, 52, 54, 56, 57, 58, 56, 54, 50, 46, 43, 42, 42, 45, 
50, 56, 62, 66, 70, 69, 65, 62, 57, 53, 50, 50, 53, 56, 59, 60, 
57, 52, 43, 33, 23, 11, 8, 9, 12, 18, 26, 33, 33, 29, 20, 9, 
-1, -13, -15, -12, -3, 9, 28, 39, 45, 47, 45, 40, 29, 24, 22, 24, 
31, 41, 60, 69, 76, 76, 72, 56, 43, 32, 24, 22, 24, 32, 39, 45, 
47, 46, 42, 31, 23, 13, 6, 1, 0, 4, 11, 21, 31, 39, 45, 43, 
39, 33, 29, 30, 36, 46, 57, 68, 77, 83, 81, 75, 67, 59, 51, 47, 
47, 51, 59, 67, 79, 84, 85, 80, 71, 60, 41, 32, 27, 28, 33, 44, 
62, 72, 77, 74, 66, 42, 24, 8, -4, -8, -4, 11, 25, 37, 45, 49, 
46, 33, 20, 7, -3, -10, -8, 3, 18, 36, 53, 67, 73, 68, 57, 45, 
33, 24, 21, 26, 35, 47, 58, 68, 70, 65, 56, 44, 32, 19, 17, 20, 
29, 39, 50, 60, 60, 55, 45, 32, 11, 0, -7, -9, -3, 7, 29, 43, 
55, 60, 59, 53, 38, 30, 26, 27, 33, 47, 55, 60, 62, 61, 56, 45, 
37, 29, 22, 19, 19, 27, 36, 46, 54, 57, 50, 37, 19, 1, -14, -23, 
-23, -15, -4, 9, 19, 22, 15, 2, -16, -33, -47, -56, -50, -36, -18, 1, 
17, 28, 24, 14, 0, -15, -33, -39, -40, -36, -28, -18, -6, -1, -1, -6, 
-14, -27, -45, -55, -59, -58, -53, -40, -31, -24, -19, -18, -19, -23, -26, -30, 
-33, -34, -32, -23, -15, -7, 1, 6, 2, -6, -17, -28, -37, -43, -42, -35, 
-24, -13, -3, 5, 8, 5, -1, -9, -18, -26, -28, -26, -21, -15, -10, -5, 
-4, -5, -7, -9, -11, -13, -12, -10, -7, -3, 4, 8, 13, 16, 18, 18, 
15, 12, 9, 6, 3, 1, 0, 0, 0, 0, 0, 4, 7, 10, 15, 19, 
23, 27, 28, 28, 27, 24, 22, 18, 17, 16, 16, 18, 21, 23, 24, 26, 
25, 23, 18, 14, 12, 11, 11, 13, 19, 23, 28, 31, 33, 30, 24, 16, 
7, 0, -5, -5, -2, 5, 13, 21, 28, 31, 29, 23, 15, 7, -5, -9, 
-8, -2, 6, 15, 26, 30, 28, 21, 11, -7, -18, -27, -32, -32, -27, -12, 
-1, 8, 12, 10, 1, -20, -35, -46, -52, -53, -43, -32, -21, -10, -2, 2, 
-1, -6, -14, -23, -31, -35, -34, -29, -20, -11, -4, 0, -3, -10, -19, -26, 
-32, -35, -32, -25, -17, -10, -5, -3, -5, -11, -18, -26, -35, -38, -38, -35, 
-30, -26, -22, -21, -22, -26, -29, -33, -35, -33, -31, -26, -19, -11, -5, -2, 
-1, -2, -7, -18, -25, -32, -39, -43, -45, -42, -37, -31, -26, -21, -17, -17, 
-18, -22, -25, -27, -27, -25, -22, -18, -14, -10, -3, 0, 3, 3, 1, -7, 
-12, -17, -20, -18, -14, -2, 6, 13, 19, 21, 20, 13, 8, 3, 0, -1, 
1, 5, 9, 12, 15, 16, 15, 12, 8, 4, 0, -2, -3, -1, 2, 6, 
11, 18, 22, 25, 27, 28, 29, 28, 27, 26, 26, 27, 31, 34, 39, 42, 
43, 42, 38, 32, 27, 22, 19, 18, 22, 26, 30, 32, 33, 34, 33, 33, 
34, 37, 39, 44, 45, 46, 45, 44, 45, 47, 50, 52, 51, 49, 47, 45, 
44, 44, 44, 44, 43, 41, 39, 37, 35, 35, 38, 41, 45, 49, 51, 53, 
51, 48, 45, 43, 40, 39, 39, 40, 43, 45, 47, 50, 50, 50, 48, 46, 
42, 39, 38, 38, 39, 42, 48, 52, 54, 55, 55, 54, 50, 47, 44, 40, 
39, 38, 39, 40, 43, 47, 51, 56, 58, 56, 52, 47, 41, 35, 33, 34, 
37, 41, 47, 48, 49, 47, 43, 40, 33, 29, 26, 22, 19, 16, 17, 19, 
22, 26, 31, 36, 36, 34, 32, 29, 26, 23, 23, 23, 23, 25, 25, 24, 
22, 20, 17, 14, 9, 7, 6, 7, 8, 10, 13, 15, 15, 15, 15, 14, 
14, 15, 15, 15, 17, 17, 17, 15, 13, 11, 8, 3, -1, -3, -6, -9, 
-12, -12, -10, -8, -5, -2, 3, 4, 2, 1, -2, -4, -5, -5, -4, -3, 
-1, 0, 0, 1, 1, 2, 2, 2, 2, 3, 4, 5, 8, 13, 18, 23, 
27, 30, 33, 33, 33, 34, 34, 34, 35, 35, 36, 36, 36, 36, 36, 36, 
34, 31, 28, 22, 20, 19, 19, 20, 22, 26, 30, 33, 35, 36, 36, 34, 
31, 28, 25, 24, 24, 25, 25, 24, 24, 23, 21, 20, 19, 19, 20, 22, 
27, 29, 31, 33, 34, 34, 34, 34, 35, 36, 37, 33, 29, 24, 19, 15, 
12, 13, 15, 16, 18, 18, 14, 10, 6, 2, -1, -3, -1, 2, 6, 11, 
16, 19, 20, 17, 14, 10, 7, 6, 9, 12, 17, 21, 24, 25, 24, 24, 
24, 25, 30, 33, 36, 39, 41, 41, 42, 42, 43, 43, 43, 42, 40, 38, 
37, 37, 38, 42, 44, 48, 52, 55, 56, 55, 53, 51, 50, 52, 56, 65, 
71, 75, 79, 81, 82, 83, 85, 86, 87, 88, 88, 88, 88, 87, 86, 84, 
81, 78, 75, 73, 72, 73, 73, 73, 73, 71, 67, 59, 54, 50, 47, 43, 
40, 33, 28, 23, 18, 14, 10, 9, 8, 8, 8, 7, 5, 2, -1, -5, 
-9, -13, -16, -17, -18, -19, -20, -24, -29, -35, -39, -43, -44, -42, -40, -38, 
-38, -38, -40, -45, -49, -53, -56, -60, -64, -65, -66, -65, -63, -62, -62, -62, 
-63, -63, -62, -59, -56, -54, -53, -53, -53, -56, -57, -60, -63, -65, -66, -67, 
-66, -63, -60, -56, -51, -44, -41, -38, -37, -39, -43, -48, -53, -57, -61, -63, 
-64, -63, -63, -62, -61, -59, -59, -58, -55, -53, -50, -45, -41, -38, -37, -37, 
-38, -42, -44, -47, -48, -49, -51, -51, -52, -53, -55, -57, -59, -59, -60, -59, 
-61, -64, -71, -78, -86, -93, -100, -106, -109, -111, -112, -112, -113, -112, -111, -110, 
-108, -108, -107, -108, -109, -110, -110, -110, -109, -109, -109, -109, -109, -107, -102, -95, 
-88, -81, -75, -69, -63, -61, -60, -62, -64, -66, -67, -66, -65, -64, -63, -64, 
-66, -68, -69, -70, -71, -70, -69, -68, -67, -68, -68, -71, -74, -78, -83, -87, 
-91, -91, -91, -91, -90, -90, -91, -93, -94, -94, -94, -90, -87, -82, -79, -78, 
-78, -80, -81, -82, -80, -77, -73, -68, -67, -66, -68, -70, -72, -73, -71, -68, 
-65, -61, -58, -57, -60, -64, -69, -77, -87, -91, -92, -89, -86, -81, -78, -79, 
-82, -86, -92, -101, -106, -108, -109, -109, -106, -105, -104, -105, -108, -112, -119, -123, 
-126, -127, -127, -124, -117, -110, -103, -96, -89, -84, -85, -89, -94, -99, -104, -105, 
-102, -96, -89, -81, -75, -69, -67, -66, -65, -67, -69, -70, -71, -71, -71, -72, 
-71, -70, -69, -67, -66, -65, -66, -67, -68, -70, -71, -71, -70, -66, -62, -57, 
-54, -51, -46, -40, -34, -30, -24, -8, 0, };
