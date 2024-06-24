#include "stm32l476xx.h"
uint16_t triangle_wave[1000] = {0, 7, 14, 22, 29, 37, 44, 52, 59, 67, 74, 82, 89, 96, 104, 111, 119, 126, 134, 141, 149, 156, 164, 171, 178, 186, 193, 201, 208, 216, 223, 231, 238, 246, 253, 260, 268, 275, 283, 290, 298, 305, 313, 320, 328, 335, 342, 350, 357, 365, 372, 380, 387, 395, 402, 410, 417, 424, 432, 439, 447, 454, 462, 469, 477, 484, 492, 499, 506, 514, 521, 529, 536, 544, 551, 559, 566, 574, 581, 588, 596, 603, 611, 618, 626, 633, 641, 648, 656, 663, 670, 678, 685, 693, 700, 708, 715, 723, 730, 738, 745, 753, 760, 767, 775, 782, 790, 797, 805, 812, 820, 827, 835, 842, 849, 857, 864, 872, 879, 887, 894, 902, 909, 917, 924, 931, 939, 946, 954, 961, 969, 976, 984, 991, 999, 1006, 1013, 1021, 1028, 1036, 1043, 1051, 1058, 1066, 1073, 1081, 1088, 1095, 1103, 1110, 1118, 1125, 1133, 1140, 1148, 1155, 1163, 1170, 1177, 1185, 1192, 1200, 1207, 1215, 1222, 1230, 1237, 1245, 1252, 1259, 1267, 1274, 1282, 1289, 1297, 1304, 1312, 1319, 1327, 1334, 1341, 1349, 1356, 1364, 1371, 1379, 1386, 1394, 1401, 1409, 1416, 1423, 1431, 1438, 1446, 1453, 1461, 1468, 1476, 1483, 1491, 1498, 1506, 1513, 1520, 1528, 1535, 1543, 1550, 1558, 1565, 1573, 1580, 1588, 1595, 1602, 1610, 1617, 1625, 1632, 1640, 1647, 1655, 1662, 1670, 1677, 1684, 1692, 1699, 1707, 1714, 1722, 1729, 1737, 1744, 1752, 1759, 1766, 1774, 1781, 1789, 1796, 1804, 1811, 1819, 1826, 1834, 1841, 1848, 1856, 1863, 1871, 1878, 1886, 1893, 1901, 1908, 1916, 1923, 1930, 1938, 1945, 1953, 1960, 1968, 1975, 1983, 1990, 1998, 2005, 2012, 2020, 2027, 2035, 2042, 2050, 2057, 2065, 2072, 2080, 2087, 2094, 2102, 2109, 2117, 2124, 2132, 2139, 2147, 2154, 2162, 2169, 2176, 2184, 2191, 2199, 2206, 2214, 2221, 2229, 2236, 2244, 2251, 2259, 2266, 2273, 2281, 2288, 2296, 2303, 2311, 2318, 2326, 2333, 2341, 2348, 2355, 2363, 2370, 2378, 2385, 2393, 2400, 2408, 2415, 2423, 2430, 2437, 2445, 2452, 2460, 2467, 2475, 2482, 2490, 2497, 2505, 2512, 2519, 2527, 2534, 2542, 2549, 2557, 2564, 2572, 2579, 2587, 2594, 2601, 2609, 2616, 2624, 2631, 2639, 2646, 2654, 2661, 2669, 2676, 2683, 2691, 2698, 2706, 2713, 2721, 2728, 2736, 2743, 2751, 2758, 2765, 2773, 2780, 2788, 2795, 2803, 2810, 2818, 2825, 2833, 2840, 2847, 2855, 2862, 2870, 2877, 2885, 2892, 2900, 2907, 2915, 2922, 2929, 2937, 2944, 2952, 2959, 2967, 2974, 2982, 2989, 2997, 3004, 3012, 3019, 3026, 3034, 3041, 3049, 3056, 3064, 3071, 3079, 3086, 3094, 3101, 3108, 3116, 3123, 3131, 3138, 3146, 3153, 3161, 3168, 3176, 3183, 3190, 3198, 3205, 3213, 3220, 3228, 3235, 3243, 3250, 3258, 3265, 3272, 3280, 3287, 3295, 3302, 3310, 3317, 3325, 3332, 3340, 3347, 3354, 3362, 3369, 3377, 3384, 3392, 3399, 3407, 3414, 3422, 3429, 3436, 3444, 3451, 3459, 3466, 3474, 3481, 3489, 3496, 3504, 3511, 3518, 3526, 3533, 3541, 3548, 3556, 3563, 3571, 3578, 3586, 3593, 3600, 3608, 3615, 3623, 3630, 3638, 3645, 3653, 3660, 3668, 3675, 3682, 3690, 3697, 3705, 3712, 3720, 3720, 3712, 3705, 3697, 3690, 3682, 3675, 3668, 3660, 3653, 3645, 3638, 3630, 3623, 3615, 3608, 3600, 3593, 3586, 3578, 3571, 3563, 3556, 3548, 3541, 3533, 3526, 3518, 3511, 3504, 3496, 3489, 3481, 3474, 3466, 3459, 3451, 3444, 3436, 3429, 3422, 3414, 3407, 3399, 3392, 3384, 3377, 3369, 3362, 3354, 3347, 3340, 3332, 3325, 3317, 3310, 3302, 3295, 3287, 3280, 3272, 3265, 3258, 3250, 3243, 3235, 3228, 3220, 3213, 3205, 3198, 3190, 3183, 3176, 3168, 3161, 3153, 3146, 3138, 3131, 3123, 3116, 3108, 3101, 3094, 3086, 3079, 3071, 3064, 3056, 3049, 3041, 3034, 3026, 3019, 3012, 3004, 2997, 2989, 2982, 2974, 2967, 2959, 2952, 2944, 2937, 2929, 2922, 2915, 2907, 2900, 2892, 2885, 2877, 2870, 2862, 2855, 2847, 2840, 2833, 2825, 2818, 2810, 2803, 2795, 2788, 2780, 2773, 2765, 2758, 2751, 2743, 2736, 2728, 2721, 2713, 2706, 2698, 2691, 2683, 2676, 2669, 2661, 2654, 2646, 2639, 2631, 2624, 2616, 2609, 2601, 2594, 2587, 2579, 2572, 2564, 2557, 2549, 2542, 2534, 2527, 2519, 2512, 2505, 2497, 2490, 2482, 2475, 2467, 2460, 2452, 2445, 2437, 2430, 2423, 2415, 2408, 2400, 2393, 2385, 2378, 2370, 2363, 2355, 2348, 2341, 2333, 2326, 2318, 2311, 2303, 2296, 2288, 2281, 2273, 2266, 2259, 2251, 2244, 2236, 2229, 2221, 2214, 2206, 2199, 2191, 2184, 2176, 2169, 2162, 2154, 2147, 2139, 2132, 2124, 2117, 2109, 2102, 2094, 2087, 2080, 2072, 2065, 2057, 2050, 2042, 2035, 2027, 2020, 2012, 2005, 1998, 1990, 1983, 1975, 1968, 1960, 1953, 1945, 1938, 1930, 1923, 1916, 1908, 1901, 1893, 1886, 1878, 1871, 1863, 1856, 1848, 1841, 1834, 1826, 1819, 1811, 1804, 1796, 1789, 1781, 1774, 1766, 1759, 1752, 1744, 1737, 1729, 1722, 1714, 1707, 1699, 1692, 1684, 1677, 1670, 1662, 1655, 1647, 1640, 1632, 1625, 1617, 1610, 1602, 1595, 1588, 1580, 1573, 1565, 1558, 1550, 1543, 1535, 1528, 1520, 1513, 1506, 1498, 1491, 1483, 1476, 1468, 1461, 1453, 1446, 1438, 1431, 1423, 1416, 1409, 1401, 1394, 1386, 1379, 1371, 1364, 1356, 1349, 1341, 1334, 1327, 1319, 1312, 1304, 1297, 1289, 1282, 1274, 1267, 1259, 1252, 1245, 1237, 1230, 1222, 1215, 1207, 1200, 1192, 1185, 1177, 1170, 1163, 1155, 1148, 1140, 1133, 1125, 1118, 1110, 1103, 1095, 1088, 1081, 1073, 1066, 1058, 1051, 1043, 1036, 1028, 1021, 1013, 1006, 999, 991, 984, 976, 969, 961, 954, 946, 939, 931, 924, 917, 909, 902, 894, 887, 879, 872, 864, 857, 849, 842, 835, 827, 820, 812, 805, 797, 790, 782, 775, 767, 760, 753, 745, 738, 730, 723, 715, 708, 700, 693, 685, 678, 670, 663, 656, 648, 641, 633, 626, 618, 611, 603, 596, 588, 581, 574, 566, 559, 551, 544, 536, 529, 521, 514, 506, 499, 492, 484, 477, 469, 462, 454, 447, 439, 432, 424, 417, 410, 402, 395, 387, 380, 372, 365, 357, 350, 342, 335, 328, 320, 313, 305, 298, 290, 283, 275, 268, 260, 253, 246, 238, 231, 223, 216, 208, 201, 193, 186, 178, 171, 164, 156, 149, 141, 134, 126, 119, 111, 104, 96, 89, 82, 74, 67, 59, 52, 44, 37, 29, 22, 14, 7, 0};
uint16_t sawtooth_wave[1000] = {0, 3, 7, 11, 14, 18, 22, 26, 29, 33, 37, 41, 44, 48, 52, 55, 59, 63, 67, 70, 74, 78, 82, 85, 89, 93, 96, 100, 104, 108, 111, 115, 119, 123, 126, 130, 134, 137, 141, 145, 149, 152, 156, 160, 164, 167, 171, 175, 178, 182, 186, 190, 193, 197, 201, 205, 208, 212, 216, 219, 223, 227, 231, 234, 238, 242, 246, 249, 253, 257, 260, 264, 268, 272, 275, 279, 283, 287, 290, 294, 298, 301, 305, 309, 313, 316, 320, 324, 328, 331, 335, 339, 342, 346, 350, 354, 357, 361, 365, 369, 372, 376, 380, 383, 387, 391, 395, 398, 402, 406, 410, 413, 417, 421, 424, 428, 432, 436, 439, 443, 447, 451, 454, 458, 462, 465, 469, 473, 477, 480, 484, 488, 492, 495, 499, 503, 506, 510, 514, 518, 521, 525, 529, 533, 536, 540, 544, 547, 551, 555, 559, 562, 566, 570, 574, 577, 581, 585, 588, 592, 596, 600, 603, 607, 611, 615, 618, 622, 626, 629, 633, 637, 641, 644, 648, 652, 656, 659, 663, 667, 670, 674, 678, 682, 685, 689, 693, 697, 700, 704, 708, 711, 715, 719, 723, 726, 730, 734, 738, 741, 745, 749, 753, 756, 760, 764, 767, 771, 775, 779, 782, 786, 790, 794, 797, 801, 805, 808, 812, 816, 820, 823, 827, 831, 835, 838, 842, 846, 849, 853, 857, 861, 864, 868, 872, 876, 879, 883, 887, 890, 894, 898, 902, 905, 909, 913, 917, 920, 924, 928, 931, 935, 939, 943, 946, 950, 954, 958, 961, 965, 969, 972, 976, 980, 984, 987, 991, 995, 999, 1002, 1006, 1010, 1013, 1017, 1021, 1025, 1028, 1032, 1036, 1040, 1043, 1047, 1051, 1054, 1058, 1062, 1066, 1069, 1073, 1077, 1081, 1084, 1088, 1092, 1095, 1099, 1103, 1107, 1110, 1114, 1118, 1122, 1125, 1129, 1133, 1136, 1140, 1144, 1148, 1151, 1155, 1159, 1163, 1166, 1170, 1174, 1177, 1181, 1185, 1189, 1192, 1196, 1200, 1204, 1207, 1211, 1215, 1218, 1222, 1226, 1230, 1233, 1237, 1241, 1245, 1248, 1252, 1256, 1259, 1263, 1267, 1271, 1274, 1278, 1282, 1286, 1289, 1293, 1297, 1300, 1304, 1308, 1312, 1315, 1319, 1323, 1327, 1330, 1334, 1338, 1341, 1345, 1349, 1353, 1356, 1360, 1364, 1368, 1371, 1375, 1379, 1382, 1386, 1390, 1394, 1397, 1401, 1405, 1409, 1412, 1416, 1420, 1423, 1427, 1431, 1435, 1438, 1442, 1446, 1450, 1453, 1457, 1461, 1464, 1468, 1472, 1476, 1479, 1483, 1487, 1491, 1494, 1498, 1502, 1506, 1509, 1513, 1517, 1520, 1524, 1528, 1532, 1535, 1539, 1543, 1547, 1550, 1554, 1558, 1561, 1565, 1569, 1573, 1576, 1580, 1584, 1588, 1591, 1595, 1599, 1602, 1606, 1610, 1614, 1617, 1621, 1625, 1629, 1632, 1636, 1640, 1643, 1647, 1651, 1655, 1658, 1662, 1666, 1670, 1673, 1677, 1681, 1684, 1688, 1692, 1696, 1699, 1703, 1707, 1711, 1714, 1718, 1722, 1725, 1729, 1733, 1737, 1740, 1744, 1748, 1752, 1755, 1759, 1763, 1766, 1770, 1774, 1778, 1781, 1785, 1789, 1793, 1796, 1800, 1804, 1807, 1811, 1815, 1819, 1822, 1826, 1830, 1834, 1837, 1841, 1845, 1848, 1852, 1856, 1860, 1863, 1867, 1871, 1875, 1878, 1882, 1886, 1889, 1893, 1897, 1901, 1904, 1908, 1912, 1916, 1919, 1923, 1927, 1930, 1934, 1938, 1942, 1945, 1949, 1953, 1957, 1960, 1964, 1968, 1971, 1975, 1979, 1983, 1986, 1990, 1994, 1998, 2001, 2005, 2009, 2012, 2016, 2020, 2024, 2027, 2031, 2035, 2039, 2042, 2046, 2050, 2053, 2057, 2061, 2065, 2068, 2072, 2076, 2080, 2083, 2087, 2091, 2094, 2098, 2102, 2106, 2109, 2113, 2117, 2121, 2124, 2128, 2132, 2135, 2139, 2143, 2147, 2150, 2154, 2158, 2162, 2165, 2169, 2173, 2176, 2180, 2184, 2188, 2191, 2195, 2199, 2203, 2206, 2210, 2214, 2217, 2221, 2225, 2229, 2232, 2236, 2240, 2244, 2247, 2251, 2255, 2259, 2262, 2266, 2270, 2273, 2277, 2281, 2285, 2288, 2292, 2296, 2300, 2303, 2307, 2311, 2314, 2318, 2322, 2326, 2329, 2333, 2337, 2341, 2344, 2348, 2352, 2355, 2359, 2363, 2367, 2370, 2374, 2378, 2382, 2385, 2389, 2393, 2396, 2400, 2404, 2408, 2411, 2415, 2419, 2423, 2426, 2430, 2434, 2437, 2441, 2445, 2449, 2452, 2456, 2460, 2464, 2467, 2471, 2475, 2478, 2482, 2486, 2490, 2493, 2497, 2501, 2505, 2508, 2512, 2516, 2519, 2523, 2527, 2531, 2534, 2538, 2542, 2546, 2549, 2553, 2557, 2560, 2564, 2568, 2572, 2575, 2579, 2583, 2587, 2590, 2594, 2598, 2601, 2605, 2609, 2613, 2616, 2620, 2624, 2628, 2631, 2635, 2639, 2642, 2646, 2650, 2654, 2657, 2661, 2665, 2669, 2672, 2676, 2680, 2683, 2687, 2691, 2695, 2698, 2702, 2706, 2710, 2713, 2717, 2721, 2724, 2728, 2732, 2736, 2739, 2743, 2747, 2751, 2754, 2758, 2762, 2765, 2769, 2773, 2777, 2780, 2784, 2788, 2792, 2795, 2799, 2803, 2806, 2810, 2814, 2818, 2821, 2825, 2829, 2833, 2836, 2840, 2844, 2847, 2851, 2855, 2859, 2862, 2866, 2870, 2874, 2877, 2881, 2885, 2888, 2892, 2896, 2900, 2903, 2907, 2911, 2915, 2918, 2922, 2926, 2929, 2933, 2937, 2941, 2944, 2948, 2952, 2956, 2959, 2963, 2967, 2970, 2974, 2978, 2982, 2985, 2989, 2993, 2997, 3000, 3004, 3008, 3012, 3015, 3019, 3023, 3026, 3030, 3034, 3038, 3041, 3045, 3049, 3053, 3056, 3060, 3064, 3067, 3071, 3075, 3079, 3082, 3086, 3090, 3094, 3097, 3101, 3105, 3108, 3112, 3116, 3120, 3123, 3127, 3131, 3135, 3138, 3142, 3146, 3149, 3153, 3157, 3161, 3164, 3168, 3172, 3176, 3179, 3183, 3187, 3190, 3194, 3198, 3202, 3205, 3209, 3213, 3217, 3220, 3224, 3228, 3231, 3235, 3239, 3243, 3246, 3250, 3254, 3258, 3261, 3265, 3269, 3272, 3276, 3280, 3284, 3287, 3291, 3295, 3299, 3302, 3306, 3310, 3313, 3317, 3321, 3325, 3328, 3332, 3336, 3340, 3343, 3347, 3351, 3354, 3358, 3362, 3366, 3369, 3373, 3377, 3381, 3384, 3388, 3392, 3395, 3399, 3403, 3407, 3410, 3414, 3418, 3422, 3425, 3429, 3433, 3436, 3440, 3444, 3448, 3451, 3455, 3459, 3463, 3466, 3470, 3474, 3477, 3481, 3485, 3489, 3492, 3496, 3500, 3504, 3507, 3511, 3515, 3518, 3522, 3526, 3530, 3533, 3537, 3541, 3545, 3548, 3552, 3556, 3559, 3563, 3567, 3571, 3574, 3578, 3582, 3586, 3589, 3593, 3597, 3600, 3604, 3608, 3612, 3615, 3619, 3623, 3627, 3630, 3634, 3638, 3641, 3645, 3649, 3653, 3656, 3660, 3664, 3668, 3671, 3675, 3679, 3682, 3686, 3690, 3694, 3697, 3701, 3705, 3709, 3712, 3716, 3720, 0};
uint16_t sine_wave[1000] = {1862, 1873, 1885, 1897, 1908, 1920, 1932, 1943, 1955, 1967, 1979, 1990, 2002, 2014, 2025, 2037, 2049, 2060, 2072, 2083, 2095, 2107, 2118, 2130, 2141, 2153, 2165, 2176, 2188, 2199, 2211, 2222, 2234, 2245, 2257, 2268, 2280, 2291, 2302, 2314, 2325, 2336, 2348, 2359, 2370, 2381, 2393, 2404, 2415, 2426, 2437, 2449, 2460, 2471, 2482, 2493, 2504, 2515, 2526, 2537, 2548, 2558, 2569, 2580, 2591, 2602, 2612, 2623, 2634, 2644, 2655, 2666, 2676, 2687, 2697, 2708, 2718, 2728, 2739, 2749, 2759, 2770, 2780, 2790, 2800, 2810, 2820, 2830, 2840, 2850, 2860, 2870, 2880, 2890, 2899, 2909, 2919, 2928, 2938, 2947, 2957, 2966, 2976, 2985, 2994, 3004, 3013, 3022, 3031, 3040, 3049, 3058, 3067, 3076, 3085, 3094, 3103, 3111, 3120, 3129, 3137, 3146, 3154, 3163, 3171, 3179, 3187, 3196, 3204, 3212, 3220, 3228, 3236, 3244, 3251, 3259, 3267, 3275, 3282, 3290, 3297, 3305, 3312, 3319, 3327, 3334, 3341, 3348, 3355, 3362, 3369, 3376, 3383, 3389, 3396, 3403, 3409, 3416, 3422, 3428, 3435, 3441, 3447, 3453, 3459, 3465, 3471, 3477, 3483, 3488, 3494, 3500, 3505, 3511, 3516, 3521, 3527, 3532, 3537, 3542, 3547, 3552, 3557, 3562, 3567, 3571, 3576, 3580, 3585, 3589, 3594, 3598, 3602, 3606, 3610, 3614, 3618, 3622, 3626, 3629, 3633, 3637, 3640, 3644, 3647, 3650, 3653, 3657, 3660, 3663, 3666, 3668, 3671, 3674, 3677, 3679, 3682, 3684, 3686, 3689, 3691, 3693, 3695, 3697, 3699, 3701, 3703, 3704, 3706, 3708, 3709, 3711, 3712, 3713, 3714, 3715, 3717, 3718, 3718, 3719, 3720, 3721, 3721, 3722, 3722, 3723, 3723, 3723, 3723, 3723, 3723, 3723, 3723, 3723, 3723, 3722, 3722, 3722, 3721, 3720, 3720, 3719, 3718, 3717, 3716, 3715, 3714, 3713, 3711, 3710, 3708, 3707, 3705, 3704, 3702, 3700, 3698, 3696, 3694, 3692, 3690, 3688, 3685, 3683, 3680, 3678, 3675, 3673, 3670, 3667, 3664, 3661, 3658, 3655, 3652, 3649, 3645, 3642, 3638, 3635, 3631, 3628, 3624, 3620, 3616, 3612, 3608, 3604, 3600, 3596, 3591, 3587, 3583, 3578, 3574, 3569, 3564, 3559, 3555, 3550, 3545, 3540, 3535, 3529, 3524, 3519, 3513, 3508, 3503, 3497, 3491, 3486, 3480, 3474, 3468, 3462, 3456, 3450, 3444, 3438, 3432, 3425, 3419, 3412, 3406, 3399, 3393, 3386, 3379, 3372, 3365, 3359, 3352, 3344, 3337, 3330, 3323, 3316, 3308, 3301, 3294, 3286, 3278, 3271, 3263, 3255, 3248, 3240, 3232, 3224, 3216, 3208, 3200, 3192, 3183, 3175, 3167, 3158, 3150, 3141, 3133, 3124, 3116, 3107, 3098, 3089, 3081, 3072, 3063, 3054, 3045, 3036, 3027, 3018, 3008, 2999, 2990, 2980, 2971, 2962, 2952, 2943, 2933, 2923, 2914, 2904, 2894, 2885, 2875, 2865, 2855, 2845, 2835, 2825, 2815, 2805, 2795, 2785, 2775, 2764, 2754, 2744, 2734, 2723, 2713, 2702, 2692, 2681, 2671, 2660, 2650, 2639, 2628, 2618, 2607, 2596, 2586, 2575, 2564, 2553, 2542, 2531, 2520, 2509, 2498, 2487, 2476, 2465, 2454, 2443, 2432, 2421, 2410, 2398, 2387, 2376, 2365, 2353, 2342, 2331, 2319, 2308, 2297, 2285, 2274, 2262, 2251, 2239, 2228, 2216, 2205, 2193, 2182, 2170, 2159, 2147, 2136, 2124, 2113, 2101, 2089, 2078, 2066, 2054, 2043, 2031, 2019, 2008, 1996, 1984, 1973, 1961, 1949, 1938, 1926, 1914, 1902, 1891, 1879, 1867, 1856, 1844, 1832, 1821, 1809, 1797, 1785, 1774, 1762, 1750, 1739, 1727, 1715, 1704, 1692, 1680, 1669, 1657, 1645, 1634, 1622, 1610, 1599, 1587, 1576, 1564, 1553, 1541, 1530, 1518, 1507, 1495, 1484, 1472, 1461, 1449, 1438, 1426, 1415, 1404, 1392, 1381, 1370, 1358, 1347, 1336, 1325, 1313, 1302, 1291, 1280, 1269, 1258, 1247, 1236, 1225, 1214, 1203, 1192, 1181, 1170, 1159, 1148, 1137, 1127, 1116, 1105, 1095, 1084, 1073, 1063, 1052, 1042, 1031, 1021, 1010, 1000, 989, 979, 969, 959, 948, 938, 928, 918, 908, 898, 888, 878, 868, 858, 848, 838, 829, 819, 809, 800, 790, 780, 771, 761, 752, 743, 733, 724, 715, 705, 696, 687, 678, 669, 660, 651, 642, 634, 625, 616, 607, 599, 590, 582, 573, 565, 556, 548, 540, 531, 523, 515, 507, 499, 491, 483, 475, 468, 460, 452, 445, 437, 429, 422, 415, 407, 400, 393, 386, 379, 371, 364, 358, 351, 344, 337, 330, 324, 317, 311, 304, 298, 291, 285, 279, 273, 267, 261, 255, 249, 243, 237, 232, 226, 220, 215, 210, 204, 199, 194, 188, 183, 178, 173, 168, 164, 159, 154, 149, 145, 140, 136, 132, 127, 123, 119, 115, 111, 107, 103, 99, 95, 92, 88, 85, 81, 78, 74, 71, 68, 65, 62, 59, 56, 53, 50, 48, 45, 43, 40, 38, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10, 9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 8, 9, 10, 11, 12, 14, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 37, 39, 41, 44, 46, 49, 52, 55, 57, 60, 63, 66, 70, 73, 76, 79, 83, 86, 90, 94, 97, 101, 105, 109, 113, 117, 121, 125, 129, 134, 138, 143, 147, 152, 156, 161, 166, 171, 176, 181, 186, 191, 196, 202, 207, 212, 218, 223, 229, 235, 240, 246, 252, 258, 264, 270, 276, 282, 288, 295, 301, 307, 314, 320, 327, 334, 340, 347, 354, 361, 368, 375, 382, 389, 396, 404, 411, 418, 426, 433, 441, 448, 456, 464, 472, 479, 487, 495, 503, 511, 519, 527, 536, 544, 552, 560, 569, 577, 586, 594, 603, 612, 620, 629, 638, 647, 656, 665, 674, 683, 692, 701, 710, 719, 729, 738, 747, 757, 766, 776, 785, 795, 804, 814, 824, 833, 843, 853, 863, 873, 883, 893, 903, 913, 923, 933, 943, 953, 964, 974, 984, 995, 1005, 1015, 1026, 1036, 1047, 1057, 1068, 1079, 1089, 1100, 1111, 1121, 1132, 1143, 1154, 1165, 1175, 1186, 1197, 1208, 1219, 1230, 1241, 1252, 1263, 1274, 1286, 1297, 1308, 1319, 1330, 1342, 1353, 1364, 1375, 1387, 1398, 1409, 1421, 1432, 1443, 1455, 1466, 1478, 1489, 1501, 1512, 1524, 1535, 1547, 1558, 1570, 1582, 1593, 1605, 1616, 1628, 1640, 1651, 1663, 1674, 1686, 1698, 1709, 1721, 1733, 1744, 1756, 1768, 1780, 1791, 1803, 1815, 1826, 1838, 1850, 1861};