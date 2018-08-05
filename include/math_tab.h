//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : math_tab.h
//  Description: common constants definition 
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ... 
//  ...
//--------------------------------------------------------------------------------------

#ifndef MATH_TAB_H
#define MATH_TAB_H


const INT16 sin_tab[] =
{
       0,     142,     285,     428,     571,     714,     857,    1000,    1143,    1286,    1429,    1572,    1714,    1857,    2000,    2143,    2285,    2428,    2570,    2713,
    2855,    2998,    3140,    3282,    3425,    3567,    3709,    3851,    3993,    4135,    4276,    4418,    4560,    4701,    4843,    4984,    5125,    5267,    5408,    5549,
    5689,    5830,    5971,    6111,    6252,    6392,    6532,    6672,    6812,    6952,    7092,    7231,    7370,    7510,    7649,    7788,    7927,    8065,    8204,    8342,
    8480,    8618,    8756,    8894,    9031,    9169,    9306,    9443,    9580,    9716,    9853,    9989,   10125,   10261,   10397,   10532,   10667,   10802,   10937,   11072,
   11206,   11341,   11475,   11609,   11742,   11876,   12009,   12142,   12274,   12407,   12539,   12671,   12803,   12934,   13065,   13196,   13327,   13458,   13588,   13718,
   13847,   13977,   14106,   14235,   14364,   14492,   14620,   14748,   14875,   15003,   15130,   15256,   15383,   15509,   15635,   15760,   15885,   16010,   16135,   16259,
   16383,   16507,   16630,   16753,   16876,   16998,   17120,   17242,   17363,   17484,   17605,   17726,   17846,   17965,   18085,   18204,   18323,   18441,   18559,   18677,
   18794,   18911,   19027,   19144,   19259,   19375,   19490,   19605,   19719,   19833,   19947,   20060,   20173,   20285,   20397,   20509,   20620,   20731,   20842,   20952,
   21062,   21171,   21280,   21388,   21497,   21604,   21712,   21818,   21925,   22031,   22137,   22242,   22347,   22451,   22555,   22658,   22761,   22864,   22966,   23068,
   23169,   23270,   23371,   23471,   23570,   23669,   23768,   23866,   23964,   24061,   24158,   24254,   24350,   24446,   24541,   24635,   24729,   24823,   24916,   25008,
   25100,   25192,   25283,   25374,   25464,   25554,   25643,   25732,   25820,   25908,   25995,   26082,   26168,   26254,   26339,   26424,   26509,   26592,   26676,   26758,
   26841,   26922,   27004,   27084,   27165,   27244,   27323,   27402,   27480,   27558,   27635,   27711,   27787,   27863,   27938,   28012,   28086,   28160,   28233,   28305,
   28377,
};
//按线数 90度 对应 1024/4＝ 256线 tan45 = 1  2^7
const INT16 tan_tab[] =
{
       0,       0,       1,       2,       3,       3,       4,       5,       6,       7,       7,       8,       9,      10,      11,      11,      12,      13,      14,      14,
      15,      16,      17,      18,      18,      19,      20,      21,      22,      23,      23,      24,      25,      26,      27,      27,      28,      29,      30,      31,
      32,      32,      33,      34,      35,      36,      37,      37,      38,      39,      40,      41,      42,      43,      44,      44,      45,      46,      47,      48,
      49,      50,      51,      52,      53,      53,      54,      55,      56,      57,      58,      59,      60,      61,      62,      63,      64,      65,      66,      67,
      68,      69,      70,      71,      72,      73,      74,      75,      76,      77,      78,      79,      81,      82,      83,      84,      85,      86,      87,      88,
      90,      91,      92,      93,      94,      96,      97,      98,      99,     101,     102,     103,     105,     106,     107,     109,     110,     111,     113,     114,
     116,     117,     118,     120,     121,     123,     124,     126,     127,     129,     131,     132,     134,     136,     137,     139,     141,     142,     144,     146,
     148,     150,     152,     154,     155,     157,     159,     161,     164,     166,     168,     170,     172,     174,     177,     179,     181,     184,     186,     189,
     191,     194,     196,     199,     202,     204,     207,     210,     213,     216,     219,     222,     225,     229,     232,     235,     239,     243,     246,     250,
     254,     258,     262,     266,     270,     274,     279,     284,     288,     293,     298,     303,     309,     314,     320,     325,     331,     337,     344,     350,
     357,     364,     372,     379,     387,     395,     404,     412,     421,     431,     441,     451,     462,     473,     485,     498,     511,     524,     538,     554,
     570,     586,     604,     623,     643,     664,     687,     711,     737,     765,     795,     827,     862,     900,     942,     987,    1037,    1092,    1154,    1222,
    1299,    1386,    1486,    1601,    1735,    1893,    2083,    2315,    2605,    2978,    3475,    4170,    5214,    6952,   10429,   20860,
};

const INT16 cos_tab[] =
{
     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,     128,
     128,     129,     129,     129,     129,     129,     129,     129,     129,     130,     130,     130,     130,     130,     130,     131,     131,     131,     131,     131,
     131,     132,     132,     132,     132,     133,     133,     133,     133,     134,     134,     134,     134,     135,     135,     135,     135,     136,     136,     136,
     137,     137,     137,     138,     138,     138,     139,     139,     140,     140,     140,     141,     141,     142,     142,     142,     143,     143,     144,     144,
     145,     145,     146,     146,     147,     147,     148,     148,     149,     149,     150,     150,     151,     152,     152,     153,     153,     154,     155,     155,
     156,     157,     157,     158,     159,     160,     160,     161,     162,     163,     163,     164,     165,     166,     167,     168,     169,     169,     170,     171,
     172,     173,     174,     175,     176,     177,     178,     179,     181,     182,     183,     184,     185,     186,     188,     189,     190,     191,     193,     194,
     195,     197,     198,     200,     201,     203,     204,     206,     208,     209,     211,     213,     214,     216,     218,     220,     222,     224,     226,     228,
     230,     232,     234,     236,     239,     241,     244,     246,     248,     251,     254,     256,     259,     262,     265,     268,     271,     274,     277,     281,
     284,     288,     291,     295,     299,     303,     307,     311,     315,     320,     324,     329,     334,     339,     344,     350,     355,     361,     367,     373,
     379,     386,     393,     400,     408,     415,     423,     432,     440,     450,     459,     469,     479,     490,     502,     514,     526,     540,     553,     568,
     584,     600,     618,     636,     656,     677,     699,     723,     748,     776,     805,     837,     872,     910,     951,     996,    1045,    1100,    1161,    1229,
    1305,    1392,    1491,    1606,    1739,    1897,    2087,    2319,    2608,    2981,    3477,    4172,    5215,    6953,   10430,   20860,
};


const INT16 atan_tab[129] =
{
  0,
  2,  4,  5,  7,  9, 11, 13, 14, 16, 18, 20, 21, 23, 25, 27, 29,
 30, 32, 34, 36, 37, 39, 41, 42, 44, 46, 48, 49, 51, 53, 54, 56,
 58, 60, 61, 63, 64, 66, 68, 69, 71, 73, 74, 76, 77, 79, 81, 82,
 84, 85, 87, 88, 90, 91, 93, 95, 96, 98, 99,100,102,103,105,106,
108,109,111,112,113,115,116,117,119,120,121,123,124,125,127,128,
129,131,132,133,134,136,137,138,139,140,142,143,144,145,146,147,
149,150,151,152,153,154,155,156,157,159,160,161,162,163,164,165,
166,167,168,169,170,171,172,173,174,175,175,176,177,178,179,180
};


#endif

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
