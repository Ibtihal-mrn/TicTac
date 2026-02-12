#include "control.h"
#include "motors.h"   // pour baseSpeed, Kp, trimL, trimR

void control_computeSpeeds(long dL, long dR, int &speedL, int &speedR) {
  // erreur basée sur la vitesse, PAS la position totale
  long error = dL - dR;

  int correction = error * Kp;

  // limite la correction pour éviter les crashs
  correction = constrain(correction, -60, 60);

  speedL = constrain(baseSpeed - correction + trimL, 0, 255);
  speedR = constrain(baseSpeed + correction + trimR, 0, 255);
}


// --------- TUNING (démarre avec ça) ----------
static const int PWM_MAX = 255;
static const int PWM_MIN = 40;      // PWM mini qui fait bouger (à ajuster)
static const int RAMP_STEP = 6;     // PWM par cycle (si dt=10ms => ~300 PWM/s)

static const float K_HEADING = 0.03;  // gain cap (ticks -> "ticks/s" de correction)

static const float KP_VEL_L = 0.25;
static const float KI_VEL_L = 1.20;

static const float KP_VEL_R = 0.25;
static const float KI_VEL_R = 1.20;

static const float I_CLAMP = 300.0;

// La droite est ~0.15% plus "rapide" en ticks -> on la réduit un peu
static const float VEL_TRIM_L = 1.0000f;
static const float VEL_TRIM_R = 0.9985f;  // = 1 / 1.0015 environ


// facteur grossier pour convertir PWM -> vitesse cible (ticks/s)
// tu pourras le recalibrer ensuite
static const float PWM_TO_TICKS_PER_SEC = 8.0;

// --------------------------------------------

static inline int sgn_int(int x) { return (x > 0) - (x < 0); }

void control_reset(DrivePIState &st) {
  st.iL = 0;
  st.iR = 0;
  st.pwmBase = 0;
}

void control_driveStraight_PI(
  DrivePIState &st,
  long headingErrTicks,
  long dL, long dR,
  int pwmTargetBase,
  float dt,
  int &pwmL, int &pwmR
) {
  // ---- rampe sur la base PWM ----
  int errRamp = pwmTargetBase - st.pwmBase;
  if (errRamp != 0) {
    st.pwmBase += sgn_int(errRamp) * min(abs(errRamp), RAMP_STEP);
  }

  // ---- vitesses mesurées en ticks/s ----
  float vL = dL / dt;
  float vR = dR / dt;

  // ---- boucle CAP : correction basée sur erreur cumulée ----
  float headingCorr = K_HEADING * (float)headingErrTicks;  // en "ticks/s"

  // ---- vitesse cible base ----
  float vBase = st.pwmBase * PWM_TO_TICKS_PER_SEC;

  float vL_ref = vBase - headingCorr;
  float vR_ref = vBase + headingCorr;

  vL_ref *= VEL_TRIM_L;
  vR_ref *= VEL_TRIM_R;

  // ---- PI vitesse gauche ----
  float eL = vL_ref - vL;
  st.iL += eL * dt;
  st.iL = constrain(st.iL, -I_CLAMP, I_CLAMP);
  float uL = KP_VEL_L * eL + KI_VEL_L * st.iL;

  // ---- PI vitesse droite ----
  float eR = vR_ref - vR;
  st.iR += eR * dt;
  st.iR = constrain(st.iR, -I_CLAMP, I_CLAMP);
  float uR = KP_VEL_R * eR + KI_VEL_R * st.iR;

  // ---- PWM final : base + PI + trim ----
  pwmL = (int)(st.pwmBase + uL + trimL);
  pwmR = (int)(st.pwmBase + uR + trimR);

  pwmL = constrain(pwmL, 0, PWM_MAX);
  pwmR = constrain(pwmR, 0, PWM_MAX);

  if (pwmL > 0) pwmL = max(pwmL, PWM_MIN);
  if (pwmR > 0) pwmR = max(pwmR, PWM_MIN);

  
}