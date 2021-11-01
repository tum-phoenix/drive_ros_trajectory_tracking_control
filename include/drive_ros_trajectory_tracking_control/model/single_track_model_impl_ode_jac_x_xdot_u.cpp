/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) single_track_model_impl_ode_jac_x_xdot_u_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_trans CASADI_PREFIX(trans)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[9] = {0, 3, 6, 12, 18, 23, 24, 25, 26};
static const casadi_int casadi_s1[4] = {1, 4, 10, 16};
static const casadi_int casadi_s2[4] = {2, 5, 11, 17};
static const casadi_int casadi_s3[4] = {7, 13, 19, 22};
static const casadi_int casadi_s4[40] = {10, 10, 0, 0, 1, 5, 9, 13, 17, 20, 23, 25, 27, 0, 0, 2, 3, 4, 0, 2, 3, 4, 1, 2, 3, 4, 2, 3, 4, 5, 2, 3, 4, 2, 3, 4, 6, 8, 7, 9};
static const casadi_int casadi_s5[40] = {10, 10, 0, 3, 4, 10, 16, 22, 23, 24, 25, 26, 27, 1, 2, 3, 4, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 7, 5, 8, 9, 8, 9};
static const casadi_int casadi_s6[23] = {10, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s7[9] = {10, 3, 0, 1, 2, 3, 5, 8, 9};
static const casadi_int casadi_s8[16] = {3, 10, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 3, 0, 1, 2};
static const casadi_int casadi_s9[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s10[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s11[5] = {1, 1, 0, 1, 0};

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

/* single_track_model_impl_ode_jac_x_xdot_u:(i0[10],i1[10],i2[3],i3)->(o0[10x10,27nz],o1[10x10,10nz],o2[10x3,3nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real *w0=w+0, *w1=w+27, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, w26, w27, w28, w29, w30, w32, w33, w34, *w35=w+69, *w36=w+78, w37, w38, w39, w40, w41, w42, *w48=w+93, *w49=w+97, *w51=w+101, *w52=w+104, *w53=w+107, *w54=w+134, *w55=w+144;
  /* #0: @0 = zeros(10x10,27nz) */
  casadi_fill(w0, 27, 0.);
  /* #1: @1 = input[0][0] */
  casadi_copy(arg[0], 10, w1);
  /* #2: @2 = @1[2] */
  for (rr=(&w2), ss=w1+2; ss!=w1+3; ss+=1) *rr++ = *ss;
  /* #3: @3 = @1[3] */
  for (rr=(&w3), ss=w1+3; ss!=w1+4; ss+=1) *rr++ = *ss;
  /* #4: @4 = @1[1] */
  for (rr=(&w4), ss=w1+1; ss!=w1+2; ss+=1) *rr++ = *ss;
  /* #5: @4 = (@3+@4) */
  w4  = (w3+w4);
  /* #6: @5 = cos(@4) */
  w5 = cos( w4 );
  /* #7: @6 = (@2*@5) */
  w6  = (w2*w5);
  /* #8: @7 = 1 */
  w7 = 1.;
  /* #9: @8 = 0.288184 */
  w8 = 2.8818443804034583e-01;
  /* #10: @9 = @1[6] */
  for (rr=(&w9), ss=w1+6; ss!=w1+7; ss+=1) *rr++ = *ss;
  /* #11: @10 = (@9-@3) */
  w10  = (w9-w3);
  /* #12: @11 = sin(@10) */
  w11 = sin( w10 );
  /* #13: @12 = 5 */
  w12 = 5.;
  /* #14: @13 = 0.15875 */
  w13 = 1.5875000000000000e-01;
  /* #15: @13 = (@13/@2) */
  w13 /= w2;
  /* #16: @13 = (@12*@13) */
  w13  = (w12*w13);
  /* #17: @14 = (@11*@13) */
  w14  = (w11*w13);
  /* #18: @15 = @1[7] */
  for (rr=(&w15), ss=w1+7; ss!=w1+8; ss+=1) *rr++ = *ss;
  /* #19: @16 = (@15-@3) */
  w16  = (w15-w3);
  /* #20: @17 = sin(@16) */
  w17 = sin( w16 );
  /* #21: @18 = 5 */
  w18 = 5.;
  /* #22: @19 = 0.17145 */
  w19 = 1.7144999999999999e-01;
  /* #23: @19 = (@19/@2) */
  w19 /= w2;
  /* #24: @19 = (@18*@19) */
  w19  = (w18*w19);
  /* #25: @20 = (@17*@19) */
  w20  = (w17*w19);
  /* #26: @14 = (@14-@20) */
  w14 -= w20;
  /* #27: @14 = (@8*@14) */
  w14  = (w8*w14);
  /* #28: @20 = -1 */
  w20 = -1.;
  /* #29: @21 = (@15-@3) */
  w21  = (w15-w3);
  /* #30: @22 = cos(@21) */
  w22 = cos( w21 );
  /* #31: @23 = (@22*@19) */
  w23  = (w22*w19);
  /* #32: @24 = (@9-@3) */
  w24  = (w9-w3);
  /* #33: @25 = cos(@24) */
  w25 = cos( w24 );
  /* #34: @26 = (@25*@13) */
  w26  = (w25*w13);
  /* #35: @23 = (@23-@26) */
  w23 -= w26;
  /* #36: @26 = 3.47 */
  w26 = 3.4700000000000002e+00;
  /* #37: @26 = (@26*@2) */
  w26 *= w2;
  /* #38: @23 = (@23/@26) */
  w23 /= w26;
  /* #39: @20 = (@20+@23) */
  w20 += w23;
  /* #40: @23 = 21.2224 */
  w23 = 2.1222410865874362e+01;
  /* #41: @27 = cos(@15) */
  w27 = cos( w15 );
  /* #42: @28 = 0.17145 */
  w28 = 1.7144999999999999e-01;
  /* #43: @19 = (@28*@19) */
  w19  = (w28*w19);
  /* #44: @19 = (@27*@19) */
  w19  = (w27*w19);
  /* #45: @19 = (-@19) */
  w19 = (- w19 );
  /* #46: @29 = cos(@9) */
  w29 = cos( w9 );
  /* #47: @30 = 0.15875 */
  w30 = 1.5875000000000000e-01;
  /* #48: @13 = (@30*@13) */
  w13  = (w30*w13);
  /* #49: @13 = (@29*@13) */
  w13  = (w29*w13);
  /* #50: @19 = (@19-@13) */
  w19 -= w13;
  /* #51: @19 = (@23*@19) */
  w19  = (w23*w19);
  /* #52: @31 = 00 */
  /* #53: @13 = 1 */
  w13 = 1.;
  /* #54: @32 = 1 */
  w32 = 1.;
  /* #55: @33 = -20 */
  w33 = -20.;
  /* #56: @34 = -20 */
  w34 = -20.;
  /* #57: @35 = vertcat(@6, @7, @14, @20, @19, @31, @13, @32, @33, @34) */
  rr=w35;
  *rr++ = w6;
  *rr++ = w7;
  *rr++ = w14;
  *rr++ = w20;
  *rr++ = w19;
  *rr++ = w13;
  *rr++ = w32;
  *rr++ = w33;
  *rr++ = w34;
  /* #58: @35 = (-@35) */
  for (i=0, rr=w35, cs=w35; i<9; ++i) *rr++ = (- *cs++ );
  /* #59: @36 = @35[:9] */
  for (rr=w36, ss=w35+0; ss!=w35+9; ss+=1) *rr++ = *ss;
  /* #60: (@0[0, 3, 6, 12, 18, 23, 24, 25, 26] = @36) */
  for (cii=casadi_s0, rr=w0, ss=w36; cii!=casadi_s0+9; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #61: @4 = sin(@4) */
  w4 = sin( w4 );
  /* #62: @31 = 00 */
  /* #63: @6 = 0.17145 */
  w6 = 1.7144999999999999e-01;
  /* #64: @7 = @1[4] */
  for (rr=(&w7), ss=w1+4; ss!=w1+5; ss+=1) *rr++ = *ss;
  /* #65: @6 = (@6*@7) */
  w6 *= w7;
  /* #66: @6 = (@6/@2) */
  w6 /= w2;
  /* #67: @14 = (@6/@2) */
  w14  = (w6/w2);
  /* #68: @14 = (@18*@14) */
  w14  = (w18*w14);
  /* #69: @20 = (@17*@14) */
  w20  = (w17*w14);
  /* #70: @19 = 0.15875 */
  w19 = 1.5875000000000000e-01;
  /* #71: @19 = (@19*@7) */
  w19 *= w7;
  /* #72: @19 = (@19/@2) */
  w19 /= w2;
  /* #73: @7 = (@19/@2) */
  w7  = (w19/w2);
  /* #74: @7 = (@12*@7) */
  w7  = (w12*w7);
  /* #75: @13 = (@11*@7) */
  w13  = (w11*w7);
  /* #76: @20 = (@20-@13) */
  w20 -= w13;
  /* #77: @20 = (@8*@20) */
  w20  = (w8*w20);
  /* #78: @13 = (@25*@7) */
  w13  = (w25*w7);
  /* #79: @32 = (@22*@14) */
  w32  = (w22*w14);
  /* #80: @13 = (@13-@32) */
  w13 -= w32;
  /* #81: @13 = (@13/@26) */
  w13 /= w26;
  /* #82: @32 = 3.47 */
  w32 = 3.4700000000000002e+00;
  /* #83: @33 = 3.47 */
  w33 = 3.4700000000000002e+00;
  /* #84: @34 = @1[5] */
  for (rr=(&w34), ss=w1+5; ss!=w1+6; ss+=1) *rr++ = *ss;
  /* #85: @33 = (@33*@34) */
  w33 *= w34;
  /* #86: @34 = cos(@9) */
  w34 = cos( w9 );
  /* #87: @37 = cos(@15) */
  w37 = cos( w15 );
  /* #88: @34 = (@34+@37) */
  w34 += w37;
  /* #89: @33 = (@33/@34) */
  w33 /= w34;
  /* #90: @37 = (@9-@3) */
  w37  = (w9-w3);
  /* #91: @38 = sin(@37) */
  w38 = sin( w37 );
  /* #92: @39 = (@33*@38) */
  w39  = (w33*w38);
  /* #93: @40 = (@15-@3) */
  w40  = (w15-w3);
  /* #94: @41 = sin(@40) */
  w41 = sin( w40 );
  /* #95: @42 = (@33*@41) */
  w42  = (w33*w41);
  /* #96: @39 = (@39+@42) */
  w39 += w42;
  /* #97: @42 = (@9-@3) */
  w42  = (w9-w3);
  /* #98: @42 = (@42-@19) */
  w42 -= w19;
  /* #99: @12 = (@12*@42) */
  w12 *= w42;
  /* #100: @42 = (@12*@25) */
  w42  = (w12*w25);
  /* #101: @39 = (@39+@42) */
  w39 += w42;
  /* #102: @42 = (@15-@3) */
  w42  = (w15-w3);
  /* #103: @42 = (@42+@6) */
  w42 += w6;
  /* #104: @18 = (@18*@42) */
  w18 *= w42;
  /* #105: @42 = (@18*@22) */
  w42  = (w18*w22);
  /* #106: @39 = (@39+@42) */
  w39 += w42;
  /* #107: @39 = (@39/@26) */
  w39 /= w26;
  /* #108: @39 = (@39/@26) */
  w39 /= w26;
  /* #109: @32 = (@32*@39) */
  w32 *= w39;
  /* #110: @13 = (@13-@32) */
  w13 -= w32;
  /* #111: @7 = (@30*@7) */
  w7  = (w30*w7);
  /* #112: @7 = (@29*@7) */
  w7  = (w29*w7);
  /* #113: @14 = (@28*@14) */
  w14  = (w28*w14);
  /* #114: @14 = (@27*@14) */
  w14  = (w27*w14);
  /* #115: @7 = (@7+@14) */
  w7 += w14;
  /* #116: @7 = (@23*@7) */
  w7  = (w23*w7);
  /* #117: @43 = 00 */
  /* #118: @44 = 00 */
  /* #119: @45 = 00 */
  /* #120: @46 = 00 */
  /* #121: @47 = 00 */
  /* #122: @48 = vertcat(@4, @31, @20, @13, @7, @43, @44, @45, @46, @47) */
  rr=w48;
  *rr++ = w4;
  *rr++ = w20;
  *rr++ = w13;
  *rr++ = w7;
  /* #123: @48 = (-@48) */
  for (i=0, rr=w48, cs=w48; i<4; ++i) *rr++ = (- *cs++ );
  /* #124: @49 = @48[:4] */
  for (rr=w49, ss=w48+0; ss!=w48+4; ss+=1) *rr++ = *ss;
  /* #125: (@0[1, 4, 10, 16] = @49) */
  for (cii=casadi_s1, rr=w0, ss=w49; cii!=casadi_s1+4; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #126: @2 = (@2*@5) */
  w2 *= w5;
  /* #127: @31 = 00 */
  /* #128: @5 = (@9-@3) */
  w5  = (w9-w3);
  /* #129: @4 = sin(@5) */
  w4 = sin( w5 );
  /* #130: @20 = (@33*@4) */
  w20  = (w33*w4);
  /* #131: @3 = (@15-@3) */
  w3  = (w15-w3);
  /* #132: @13 = sin(@3) */
  w13 = sin( w3 );
  /* #133: @7 = (@33*@13) */
  w7  = (w33*w13);
  /* #134: @20 = (@20+@7) */
  w20 += w7;
  /* #135: @7 = -5 */
  w7 = -5.;
  /* #136: @14 = (@7*@11) */
  w14  = (w7*w11);
  /* #137: @10 = cos(@10) */
  w10 = cos( w10 );
  /* #138: @32 = (@12*@10) */
  w32  = (w12*w10);
  /* #139: @14 = (@14-@32) */
  w14 -= w32;
  /* #140: @20 = (@20-@14) */
  w20 -= w14;
  /* #141: @14 = -5 */
  w14 = -5.;
  /* #142: @32 = (@14*@17) */
  w32  = (w14*w17);
  /* #143: @16 = cos(@16) */
  w16 = cos( w16 );
  /* #144: @39 = (@18*@16) */
  w39  = (w18*w16);
  /* #145: @32 = (@32-@39) */
  w32 -= w39;
  /* #146: @20 = (@20-@32) */
  w20 -= w32;
  /* #147: @20 = (@8*@20) */
  w20  = (w8*w20);
  /* #148: @40 = cos(@40) */
  w40 = cos( w40 );
  /* #149: @32 = (@33*@40) */
  w32  = (w33*w40);
  /* #150: @32 = (-@32) */
  w32 = (- w32 );
  /* #151: @37 = cos(@37) */
  w37 = cos( w37 );
  /* #152: @39 = (@33*@37) */
  w39  = (w33*w37);
  /* #153: @32 = (@32-@39) */
  w32 -= w39;
  /* #154: @7 = (@7*@25) */
  w7 *= w25;
  /* #155: @24 = sin(@24) */
  w24 = sin( w24 );
  /* #156: @39 = (@12*@24) */
  w39  = (w12*w24);
  /* #157: @7 = (@7+@39) */
  w7 += w39;
  /* #158: @32 = (@32+@7) */
  w32 += w7;
  /* #159: @14 = (@14*@22) */
  w14 *= w22;
  /* #160: @21 = sin(@21) */
  w21 = sin( w21 );
  /* #161: @7 = (@18*@21) */
  w7  = (w18*w21);
  /* #162: @14 = (@14+@7) */
  w14 += w7;
  /* #163: @32 = (@32+@14) */
  w32 += w14;
  /* #164: @32 = (@32/@26) */
  w32 /= w26;
  /* #165: @14 = -0.79375 */
  w14 = -7.9374999999999996e-01;
  /* #166: @14 = (@14*@29) */
  w14 *= w29;
  /* #167: @7 = -0.85725 */
  w7 = -8.5724999999999996e-01;
  /* #168: @7 = (@7*@27) */
  w7 *= w27;
  /* #169: @14 = (@14-@7) */
  w14 -= w7;
  /* #170: @14 = (@23*@14) */
  w14  = (w23*w14);
  /* #171: @43 = 00 */
  /* #172: @44 = 00 */
  /* #173: @45 = 00 */
  /* #174: @46 = 00 */
  /* #175: @47 = 00 */
  /* #176: @49 = vertcat(@2, @31, @20, @32, @14, @43, @44, @45, @46, @47) */
  rr=w49;
  *rr++ = w2;
  *rr++ = w20;
  *rr++ = w32;
  *rr++ = w14;
  /* #177: @49 = (-@49) */
  for (i=0, rr=w49, cs=w49; i<4; ++i) *rr++ = (- *cs++ );
  /* #178: @48 = @49[:4] */
  for (rr=w48, ss=w49+0; ss!=w49+4; ss+=1) *rr++ = *ss;
  /* #179: (@0[2, 5, 11, 17] = @48) */
  for (cii=casadi_s2, rr=w0, ss=w48; cii!=casadi_s2+4; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #180: @31 = 00 */
  /* #181: @43 = 00 */
  /* #182: @5 = cos(@5) */
  w5 = cos( w5 );
  /* #183: @2 = 3.47 */
  w2 = 3.4700000000000002e+00;
  /* #184: @2 = (@2/@34) */
  w2 /= w34;
  /* #185: @20 = (@5*@2) */
  w20  = (w5*w2);
  /* #186: @3 = cos(@3) */
  w3 = cos( w3 );
  /* #187: @32 = (@3*@2) */
  w32  = (w3*w2);
  /* #188: @20 = (@20+@32) */
  w20 += w32;
  /* #189: @20 = (@8*@20) */
  w20  = (w8*w20);
  /* #190: @32 = (@38*@2) */
  w32  = (w38*w2);
  /* #191: @14 = (@41*@2) */
  w14  = (w41*w2);
  /* #192: @32 = (@32+@14) */
  w32 += w14;
  /* #193: @32 = (@32/@26) */
  w32 /= w26;
  /* #194: @14 = sin(@9) */
  w14 = sin( w9 );
  /* #195: @7 = 0.15875 */
  w7 = 1.5875000000000000e-01;
  /* #196: @39 = (@7*@2) */
  w39  = (w7*w2);
  /* #197: @39 = (@14*@39) */
  w39  = (w14*w39);
  /* #198: @42 = sin(@15) */
  w42 = sin( w15 );
  /* #199: @6 = 0.17145 */
  w6 = 1.7144999999999999e-01;
  /* #200: @2 = (@6*@2) */
  w2  = (w6*w2);
  /* #201: @2 = (@42*@2) */
  w2  = (w42*w2);
  /* #202: @39 = (@39-@2) */
  w39 -= w2;
  /* #203: @39 = (@23*@39) */
  w39  = (w23*w39);
  /* #204: @2 = -16.6667 */
  w2 = -1.6666666666666668e+01;
  /* #205: @44 = 00 */
  /* #206: @45 = 00 */
  /* #207: @46 = 00 */
  /* #208: @47 = 00 */
  /* #209: @48 = vertcat(@31, @43, @20, @32, @39, @2, @44, @45, @46, @47) */
  rr=w48;
  *rr++ = w20;
  *rr++ = w32;
  *rr++ = w39;
  *rr++ = w2;
  /* #210: @48 = (-@48) */
  for (i=0, rr=w48, cs=w48; i<4; ++i) *rr++ = (- *cs++ );
  /* #211: @49 = @48[:4] */
  for (rr=w49, ss=w48+0; ss!=w48+4; ss+=1) *rr++ = *ss;
  /* #212: (@0[7, 13, 19, 22] = @49) */
  for (cii=casadi_s3, rr=w0, ss=w49; cii!=casadi_s3+4; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #213: @31 = 00 */
  /* #214: @43 = 00 */
  /* #215: @34 = (@33/@34) */
  w34  = (w33/w34);
  /* #216: @20 = sin(@9) */
  w20 = sin( w9 );
  /* #217: @20 = (@34*@20) */
  w20  = (w34*w20);
  /* #218: @32 = (@5*@20) */
  w32  = (w5*w20);
  /* #219: @4 = (@33*@4) */
  w4  = (w33*w4);
  /* #220: @32 = (@32-@4) */
  w32 -= w4;
  /* #221: @4 = (@3*@20) */
  w4  = (w3*w20);
  /* #222: @32 = (@32+@4) */
  w32 += w4;
  /* #223: @4 = 5 */
  w4 = 5.;
  /* #224: @11 = (@4*@11) */
  w11  = (w4*w11);
  /* #225: @10 = (@12*@10) */
  w10  = (w12*w10);
  /* #226: @11 = (@11+@10) */
  w11 += w10;
  /* #227: @32 = (@32-@11) */
  w32 -= w11;
  /* #228: @32 = (@8*@32) */
  w32  = (w8*w32);
  /* #229: @11 = (@38*@20) */
  w11  = (w38*w20);
  /* #230: @37 = (@33*@37) */
  w37  = (w33*w37);
  /* #231: @11 = (@11+@37) */
  w11 += w37;
  /* #232: @37 = (@41*@20) */
  w37  = (w41*w20);
  /* #233: @11 = (@11+@37) */
  w11 += w37;
  /* #234: @4 = (@4*@25) */
  w4 *= w25;
  /* #235: @24 = (@12*@24) */
  w24  = (w12*w24);
  /* #236: @4 = (@4-@24) */
  w4 -= w24;
  /* #237: @11 = (@11+@4) */
  w11 += w4;
  /* #238: @11 = (@11/@26) */
  w11 /= w26;
  /* #239: @4 = (@7*@20) */
  w4  = (w7*w20);
  /* #240: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #241: @24 = (@7*@33) */
  w24  = (w7*w33);
  /* #242: @25 = cos(@9) */
  w25 = cos( w9 );
  /* #243: @24 = (@24*@25) */
  w24 *= w25;
  /* #244: @4 = (@4+@24) */
  w4 += w24;
  /* #245: @20 = (@6*@20) */
  w20  = (w6*w20);
  /* #246: @20 = (@42*@20) */
  w20  = (w42*w20);
  /* #247: @4 = (@4-@20) */
  w4 -= w20;
  /* #248: @20 = 0.79375 */
  w20 = 7.9374999999999996e-01;
  /* #249: @20 = (@20*@29) */
  w20 *= w29;
  /* #250: @30 = (@30*@12) */
  w30 *= w12;
  /* #251: @9 = sin(@9) */
  w9 = sin( w9 );
  /* #252: @30 = (@30*@9) */
  w30 *= w9;
  /* #253: @20 = (@20-@30) */
  w20 -= w30;
  /* #254: @4 = (@4+@20) */
  w4 += w20;
  /* #255: @4 = (@23*@4) */
  w4  = (w23*w4);
  /* #256: @44 = 00 */
  /* #257: @45 = 00 */
  /* #258: @46 = 00 */
  /* #259: @47 = 00 */
  /* #260: @50 = 00 */
  /* #261: @51 = vertcat(@31, @43, @32, @11, @4, @44, @45, @46, @47, @50) */
  rr=w51;
  *rr++ = w32;
  *rr++ = w11;
  *rr++ = w4;
  /* #262: @51 = (-@51) */
  for (i=0, rr=w51, cs=w51; i<3; ++i) *rr++ = (- *cs++ );
  /* #263: @52 = @51[:3] */
  for (rr=w52, ss=w51+0; ss!=w51+3; ss+=1) *rr++ = *ss;
  /* #264: (@0[8:26:6] = @52) */
  for (rr=w0+8, ss=w52; rr!=w0+26; rr+=6) *rr = *ss++;
  /* #265: @31 = 00 */
  /* #266: @43 = 00 */
  /* #267: @32 = sin(@15) */
  w32 = sin( w15 );
  /* #268: @34 = (@34*@32) */
  w34 *= w32;
  /* #269: @5 = (@5*@34) */
  w5 *= w34;
  /* #270: @3 = (@3*@34) */
  w3 *= w34;
  /* #271: @13 = (@33*@13) */
  w13  = (w33*w13);
  /* #272: @3 = (@3-@13) */
  w3 -= w13;
  /* #273: @5 = (@5+@3) */
  w5 += w3;
  /* #274: @3 = 5 */
  w3 = 5.;
  /* #275: @17 = (@3*@17) */
  w17  = (w3*w17);
  /* #276: @16 = (@18*@16) */
  w16  = (w18*w16);
  /* #277: @17 = (@17+@16) */
  w17 += w16;
  /* #278: @5 = (@5-@17) */
  w5 -= w17;
  /* #279: @8 = (@8*@5) */
  w8 *= w5;
  /* #280: @38 = (@38*@34) */
  w38 *= w34;
  /* #281: @41 = (@41*@34) */
  w41 *= w34;
  /* #282: @40 = (@33*@40) */
  w40  = (w33*w40);
  /* #283: @41 = (@41+@40) */
  w41 += w40;
  /* #284: @38 = (@38+@41) */
  w38 += w41;
  /* #285: @3 = (@3*@22) */
  w3 *= w22;
  /* #286: @21 = (@18*@21) */
  w21  = (w18*w21);
  /* #287: @3 = (@3-@21) */
  w3 -= w21;
  /* #288: @38 = (@38+@3) */
  w38 += w3;
  /* #289: @38 = (@38/@26) */
  w38 /= w26;
  /* #290: @7 = (@7*@34) */
  w7 *= w34;
  /* #291: @14 = (@14*@7) */
  w14 *= w7;
  /* #292: @34 = (@6*@34) */
  w34  = (w6*w34);
  /* #293: @42 = (@42*@34) */
  w42 *= w34;
  /* #294: @6 = (@6*@33) */
  w6 *= w33;
  /* #295: @33 = cos(@15) */
  w33 = cos( w15 );
  /* #296: @6 = (@6*@33) */
  w6 *= w33;
  /* #297: @42 = (@42+@6) */
  w42 += w6;
  /* #298: @14 = (@14-@42) */
  w14 -= w42;
  /* #299: @42 = 0.85725 */
  w42 = 8.5724999999999996e-01;
  /* #300: @42 = (@42*@27) */
  w42 *= w27;
  /* #301: @28 = (@28*@18) */
  w28 *= w18;
  /* #302: @15 = sin(@15) */
  w15 = sin( w15 );
  /* #303: @28 = (@28*@15) */
  w28 *= w15;
  /* #304: @42 = (@42-@28) */
  w42 -= w28;
  /* #305: @14 = (@14-@42) */
  w14 -= w42;
  /* #306: @23 = (@23*@14) */
  w23 *= w14;
  /* #307: @44 = 00 */
  /* #308: @45 = 00 */
  /* #309: @46 = 00 */
  /* #310: @47 = 00 */
  /* #311: @50 = 00 */
  /* #312: @52 = vertcat(@31, @43, @8, @38, @23, @44, @45, @46, @47, @50) */
  rr=w52;
  *rr++ = w8;
  *rr++ = w38;
  *rr++ = w23;
  /* #313: @52 = (-@52) */
  for (i=0, rr=w52, cs=w52; i<3; ++i) *rr++ = (- *cs++ );
  /* #314: @51 = @52[:3] */
  for (rr=w51, ss=w52+0; ss!=w52+3; ss+=1) *rr++ = *ss;
  /* #315: (@0[9:27:6] = @51) */
  for (rr=w0+9, ss=w51; rr!=w0+27; rr+=6) *rr = *ss++;
  /* #316: @53 = @0' */
  casadi_trans(w0,casadi_s5, w53, casadi_s4, iw);
  /* #317: output[0][0] = @53 */
  casadi_copy(w53, 27, res[0]);
  /* #318: @1 = zeros(10x10,10nz) */
  casadi_fill(w1, 10, 0.);
  /* #319: @54 = ones(10x1) */
  casadi_fill(w54, 10, 1.);
  /* #320: (@1[:10] = @54) */
  for (rr=w1+0, ss=w54; rr!=w1+10; rr+=1) *rr = *ss++;
  /* #321: @54 = @1' */
  casadi_trans(w1,casadi_s6, w54, casadi_s6, iw);
  /* #322: output[1][0] = @54 */
  casadi_copy(w54, 10, res[1]);
  /* #323: @51 = zeros(3x10,3nz) */
  casadi_fill(w51, 3, 0.);
  /* #324: @31 = 00 */
  /* #325: @43 = 00 */
  /* #326: @44 = 00 */
  /* #327: @45 = 00 */
  /* #328: @46 = 00 */
  /* #329: @8 = 16.6667 */
  w8 = 1.6666666666666668e+01;
  /* #330: @47 = 00 */
  /* #331: @50 = 00 */
  /* #332: @38 = 20 */
  w38 = 20.;
  /* #333: @23 = 20 */
  w23 = 20.;
  /* #334: @52 = vertcat(@31, @43, @44, @45, @46, @8, @47, @50, @38, @23) */
  rr=w52;
  *rr++ = w8;
  *rr++ = w38;
  *rr++ = w23;
  /* #335: @52 = (-@52) */
  for (i=0, rr=w52, cs=w52; i<3; ++i) *rr++ = (- *cs++ );
  /* #336: @55 = @52[:3] */
  for (rr=w55, ss=w52+0; ss!=w52+3; ss+=1) *rr++ = *ss;
  /* #337: (@51[:3] = @55) */
  for (rr=w51+0, ss=w55; rr!=w51+3; rr+=1) *rr = *ss++;
  /* #338: @55 = @51' */
  casadi_trans(w51,casadi_s8, w55, casadi_s7, iw);
  /* #339: output[2][0] = @55 */
  casadi_copy(w55, 3, res[2]);
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int single_track_model_impl_ode_jac_x_xdot_u(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT void single_track_model_impl_ode_jac_x_xdot_u_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void single_track_model_impl_ode_jac_x_xdot_u_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int single_track_model_impl_ode_jac_x_xdot_u_n_in(void) { return 4;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int single_track_model_impl_ode_jac_x_xdot_u_n_out(void) { return 3;}

extern "C" CASADI_SYMBOL_EXPORT const char* single_track_model_impl_ode_jac_x_xdot_u_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* single_track_model_impl_ode_jac_x_xdot_u_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* single_track_model_impl_ode_jac_x_xdot_u_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s9;
    case 1: return casadi_s9;
    case 2: return casadi_s10;
    case 3: return casadi_s11;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* single_track_model_impl_ode_jac_x_xdot_u_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s6;
    case 2: return casadi_s7;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int single_track_model_impl_ode_jac_x_xdot_u_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 14;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 11;
  if (sz_w) *sz_w = 147;
  return 0;
}

