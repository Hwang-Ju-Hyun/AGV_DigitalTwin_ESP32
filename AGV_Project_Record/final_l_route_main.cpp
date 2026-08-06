// ESP32 AGV final verified autonomous L-route code
// Route: forward 30 cm -> clockwise 90 deg -> forward 30 cm
#include <Arduino.h>

// TB6612 pins
const int AIN1 = 25;
const int AIN2 = 26;
const int PWMA = 27;
const int BIN1 = 33;
const int BIN2 = 32;
const int PWMB = 14;
const int STBY = 13;

// Encoders and BOOT button
const int LEFT_ENC_A = 19;
const int LEFT_ENC_B = 18;
const int RIGHT_ENC_A = 17;
const int RIGHT_ENC_B = 16;
const int BOOT_BUTTON = 0;

// PWM
const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;

// Verified distances
const int32_t FORWARD_30CM_COUNT = 520;
const int32_t CLOCKWISE_90_COUNT = 176;

// Forward speed profile
const int FORWARD_LEFT_START_PWM = 50;
const int FORWARD_RIGHT_START_PWM = 55;
const int FORWARD_LEFT_CRUISE_PWM = 80;
const int FORWARD_RIGHT_CRUISE_PWM = 85;
const int32_t FORWARD_ACCEL_COUNTS = 100;
const int32_t FORWARD_DECEL_COUNTS = 160;

// Turn speed profile
const int TURN_LEFT_CRUISE_PWM = 58;
const int TURN_RIGHT_CRUISE_PWM = 63;
const int TURN_LEFT_MID_PWM = 50;
const int TURN_RIGHT_MID_PWM = 55;
const int TURN_LEFT_SLOW_PWM = 44;
const int TURN_RIGHT_SLOW_PWM = 49;

const int FORWARD_MIN_PWM = 42;
const int TURN_MIN_PWM = 36;
const int FORWARD_MAX_PWM = 100;
const int TURN_MAX_PWM = 90;

const float FORWARD_SYNC_KP = 0.25f;
const float TURN_SYNC_KP = 0.20f;

const unsigned long COUNTDOWN_MS = 5000;
const unsigned long PAUSE_MS = 1000;
const unsigned long FORWARD_TIMEOUT_MS = 10000;
const unsigned long TURN_TIMEOUT_MS = 6000;
const unsigned long PROGRESS_CHECK_MS = 300;
const int MAX_NO_PROGRESS_WINDOWS = 4;

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

enum MotionType {
    MOTION_NONE,
    MOTION_FORWARD,
    MOTION_CLOCKWISE
};

enum RouteState {
    ROUTE_READY,
    ROUTE_COUNTDOWN,
    ROUTE_FORWARD_1,
    ROUTE_PAUSE_1,
    ROUTE_TURN,
    ROUTE_PAUSE_2,
    ROUTE_FORWARD_2
};

enum MotionResult {
    MOTION_RUNNING,
    MOTION_COMPLETE,
    MOTION_FAILED
};

RouteState routeState = ROUTE_READY;
MotionType motionType = MOTION_NONE;
int32_t motionTargetCount = 0;

unsigned long countdownStartMs = 0;
unsigned long pauseStartMs = 0;
unsigned long motionStartMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastProgressCheckMs = 0;

int32_t lastProgressLeft = 0;
int32_t lastProgressRight = 0;
int leftNoProgressWindows = 0;
int rightNoProgressWindows = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
bool buttonArmed = true;
unsigned long lastDebounceMs = 0;
const unsigned long DEBOUNCE_MS = 50;

void IRAM_ATTR leftEncoderISR()
{
    int b = digitalRead(LEFT_ENC_B);
    portENTER_CRITICAL_ISR(&encoderMux);
    if (b == HIGH)
        leftCount--;
    else
        leftCount++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR rightEncoderISR()
{
    int b = digitalRead(RIGHT_ENC_B);
    portENTER_CRITICAL_ISR(&encoderMux);
    if (b == HIGH)
        rightCount++;
    else
        rightCount--;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void readEncoderCounts(int32_t &left, int32_t &right)
{
    portENTER_CRITICAL(&encoderMux);
    left = leftCount;
    right = rightCount;
    portEXIT_CRITICAL(&encoderMux);
}

void resetEncoderCounts()
{
    portENTER_CRITICAL(&encoderMux);
    leftCount = 0;
    rightCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}

void writeMotorPWM(int leftPWM, int rightPWM)
{
    leftPWM = constrain(leftPWM, 0, FORWARD_MAX_PWM);
    rightPWM = constrain(rightPWM, 0, FORWARD_MAX_PWM);
    ledcWrite(LEFT_PWM_CHANNEL, leftPWM);
    ledcWrite(RIGHT_PWM_CHANNEL, rightPWM);
}

void disableMotors()
{
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
    digitalWrite(STBY, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void setMotionDirection(MotionType type)
{
    if (type == MOTION_FORWARD)
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    else if (type == MOTION_CLOCKWISE)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
}

void getMotionProgress(
    int32_t leftRaw,
    int32_t rightRaw,
    int32_t &leftProgress,
    int32_t &rightProgress)
{
    if (motionType == MOTION_FORWARD)
    {
        leftProgress = leftRaw;
        rightProgress = rightRaw;
    }
    else
    {
        leftProgress = -leftRaw;
        rightProgress = rightRaw;
    }
}

void abortRoute(const char *reason)
{
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
    delay(100);
    disableMotors();
    motionType = MOTION_NONE;
    routeState = ROUTE_READY;

    Serial.println();
    Serial.println("================================");
    Serial.println(reason);
    Serial.println("ROUTE ABORTED - MOTORS STOPPED");
    Serial.println("PRESS BOOT TO TRY AGAIN");
    Serial.println("================================");
}

void startMotion(MotionType type, int32_t targetCount)
{
    disableMotors();
    resetEncoderCounts();

    motionType = type;
    motionTargetCount = targetCount;
    setMotionDirection(type);
    digitalWrite(STBY, HIGH);

    if (type == MOTION_FORWARD)
    {
        writeMotorPWM(
            FORWARD_LEFT_START_PWM,
            FORWARD_RIGHT_START_PWM);
    }
    else
    {
        writeMotorPWM(
            TURN_LEFT_CRUISE_PWM,
            TURN_RIGHT_CRUISE_PWM);
    }

    motionStartMs = millis();
    lastPrintMs = motionStartMs;
    lastProgressCheckMs = motionStartMs;
    lastProgressLeft = 0;
    lastProgressRight = 0;
    leftNoProgressWindows = 0;
    rightNoProgressWindows = 0;
}

void stopCompletedMotion()
{
    ledcWrite(LEFT_PWM_CHANNEL, 0);
    ledcWrite(RIGHT_PWM_CHANNEL, 0);
    delay(150);
    disableMotors();

    int32_t leftRaw;
    int32_t rightRaw;
    readEncoderCounts(leftRaw, rightRaw);

    int32_t leftProgress;
    int32_t rightProgress;
    getMotionProgress(
        leftRaw,
        rightRaw,
        leftProgress,
        rightProgress);

    Serial.printf(
        "SEGMENT COMPLETE L=%ld R=%ld\n",
        (long)leftProgress,
        (long)rightProgress);

    motionType = MOTION_NONE;
}

MotionResult updateMotion()
{
    int32_t leftRaw;
    int32_t rightRaw;
    readEncoderCounts(leftRaw, rightRaw);

    int32_t leftProgress;
    int32_t rightProgress;
    getMotionProgress(
        leftRaw,
        rightRaw,
        leftProgress,
        rightProgress);

    if (leftProgress < -10 || rightProgress < -10)
    {
        abortRoute("SAFETY STOP: WRONG DIRECTION");
        return MOTION_FAILED;
    }

    int32_t overLimit =
        (motionType == MOTION_FORWARD) ? 100 : 60;

    if (leftProgress > motionTargetCount + overLimit ||
        rightProgress > motionTargetCount + overLimit)
    {
        abortRoute("SAFETY STOP: COUNT OVER LIMIT");
        return MOTION_FAILED;
    }

    int32_t difference = leftProgress - rightProgress;
    if (difference < 0)
        difference = -difference;

    int32_t differenceLimit =
        (motionType == MOTION_FORWARD) ? 80 : 50;

    if (difference > differenceLimit)
    {
        abortRoute("SAFETY STOP: WHEEL COUNT MISMATCH");
        return MOTION_FAILED;
    }

    if (leftProgress >= motionTargetCount &&
        rightProgress >= motionTargetCount)
    {
        stopCompletedMotion();
        return MOTION_COMPLETE;
    }

    unsigned long timeoutMs =
        (motionType == MOTION_FORWARD)
            ? FORWARD_TIMEOUT_MS
            : TURN_TIMEOUT_MS;

    if ((millis() - motionStartMs) >= timeoutMs)
    {
        abortRoute("SAFETY STOP: MOTION TIMEOUT");
        return MOTION_FAILED;
    }

    if ((millis() - lastProgressCheckMs) >=
        PROGRESS_CHECK_MS)
    {
        int32_t leftAdvance =
            leftProgress - lastProgressLeft;
        int32_t rightAdvance =
            rightProgress - lastProgressRight;

        if (leftProgress < motionTargetCount)
        {
            if (leftAdvance < 2)
                leftNoProgressWindows++;
            else
                leftNoProgressWindows = 0;
        }
        else
        {
            leftNoProgressWindows = 0;
        }

        if (rightProgress < motionTargetCount)
        {
            if (rightAdvance < 2)
                rightNoProgressWindows++;
            else
                rightNoProgressWindows = 0;
        }
        else
        {
            rightNoProgressWindows = 0;
        }

        lastProgressLeft = leftProgress;
        lastProgressRight = rightProgress;
        lastProgressCheckMs = millis();

        if (leftNoProgressWindows >= MAX_NO_PROGRESS_WINDOWS ||
            rightNoProgressWindows >= MAX_NO_PROGRESS_WINDOWS)
        {
            abortRoute(
                "SAFETY STOP: MOTOR OR ENCODER STALL");
            return MOTION_FAILED;
        }
    }

    int baseLeftPWM;
    int baseRightPWM;
    int minPWM;
    int maxPWM;
    float syncKp;
    int correctionLimit;

    if (motionType == MOTION_FORWARD)
    {
        int32_t slowerProgress =
            min(leftProgress, rightProgress);
        int32_t leadingProgress =
            max(leftProgress, rightProgress);

        if (slowerProgress < 0)
            slowerProgress = 0;

        int32_t remaining =
            motionTargetCount - leadingProgress;
        if (remaining < 0)
            remaining = 0;

        float accelRatio =
            (float)slowerProgress /
            (float)FORWARD_ACCEL_COUNTS;
        float decelRatio =
            (float)remaining /
            (float)FORWARD_DECEL_COUNTS;

        accelRatio = constrain(accelRatio, 0.0f, 1.0f);
        decelRatio = constrain(decelRatio, 0.0f, 1.0f);

        float motionRatio = min(accelRatio, decelRatio);

        baseLeftPWM =
            FORWARD_LEFT_START_PWM +
            (int)((FORWARD_LEFT_CRUISE_PWM -
                   FORWARD_LEFT_START_PWM) *
                  motionRatio);

        baseRightPWM =
            FORWARD_RIGHT_START_PWM +
            (int)((FORWARD_RIGHT_CRUISE_PWM -
                   FORWARD_RIGHT_START_PWM) *
                  motionRatio);

        minPWM = FORWARD_MIN_PWM;
        maxPWM = FORWARD_MAX_PWM;
        syncKp = FORWARD_SYNC_KP;
        correctionLimit = 15;
    }
    else
    {
        int32_t averageProgress =
            (leftProgress + rightProgress) / 2;
        int32_t remaining =
            motionTargetCount - averageProgress;

        if (remaining > 70)
        {
            baseLeftPWM = TURN_LEFT_CRUISE_PWM;
            baseRightPWM = TURN_RIGHT_CRUISE_PWM;
        }
        else if (remaining > 25)
        {
            baseLeftPWM = TURN_LEFT_MID_PWM;
            baseRightPWM = TURN_RIGHT_MID_PWM;
        }
        else
        {
            baseLeftPWM = TURN_LEFT_SLOW_PWM;
            baseRightPWM = TURN_RIGHT_SLOW_PWM;
        }

        minPWM = TURN_MIN_PWM;
        maxPWM = TURN_MAX_PWM;
        syncKp = TURN_SYNC_KP;
        correctionLimit = 12;
    }

    int32_t syncError =
        leftProgress - rightProgress;
    int correction =
        (int)(syncError * syncKp);
    correction = constrain(
        correction,
        -correctionLimit,
        correctionLimit);

    int leftPWM = baseLeftPWM - correction;
    int rightPWM = baseRightPWM + correction;

    if (leftProgress >= motionTargetCount)
        leftPWM = 0;
    else
        leftPWM = constrain(leftPWM, minPWM, maxPWM);

    if (rightProgress >= motionTargetCount)
        rightPWM = 0;
    else
        rightPWM = constrain(rightPWM, minPWM, maxPWM);

    writeMotorPWM(leftPWM, rightPWM);

    if ((millis() - lastPrintMs) >= 100)
    {
        lastPrintMs = millis();
        Serial.printf(
            "TYPE=%d L=%ld PWM=%d | R=%ld PWM=%d\n",
            (int)motionType,
            (long)leftProgress,
            leftPWM,
            (long)rightProgress,
            rightPWM);
    }

    return MOTION_RUNNING;
}

bool bootButtonPressed()
{
    bool reading = digitalRead(BOOT_BUTTON);

    if (reading != lastButtonReading)
    {
        lastDebounceMs = millis();
        lastButtonReading = reading;
    }

    if ((millis() - lastDebounceMs) >= DEBOUNCE_MS)
    {
        if (reading != stableButtonState)
        {
            stableButtonState = reading;

            if (stableButtonState == HIGH)
            {
                buttonArmed = true;
            }
            else if (buttonArmed)
            {
                buttonArmed = false;
                return true;
            }
        }
    }

    return false;
}

void beginRouteCountdown()
{
    disableMotors();
    routeState = ROUTE_COUNTDOWN;
    countdownStartMs = millis();

    Serial.println();
    Serial.println("================================");
    Serial.println("AUTONOMOUS L ROUTE");
    Serial.println("30CM -> CW 90 -> 30CM");
    Serial.println("START IN 5 SECONDS");
    Serial.println("PRESS BOOT AGAIN TO CANCEL");
    Serial.println("================================");
}

void finishRoute()
{
    disableMotors();
    motionType = MOTION_NONE;
    routeState = ROUTE_READY;

    Serial.println();
    Serial.println("================================");
    Serial.println("AUTONOMOUS L ROUTE FINISHED");
    Serial.println("MOTORS STOPPED");
    Serial.println("PRESS BOOT TO RUN AGAIN");
    Serial.println("================================");
}

void setup()
{
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, LOW);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    pinMode(BOOT_BUTTON, INPUT_PULLUP);

    ledcSetup(
        LEFT_PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION);
    ledcSetup(
        RIGHT_PWM_CHANNEL,
        PWM_FREQUENCY,
        PWM_RESOLUTION);
    ledcAttachPin(PWMA, LEFT_PWM_CHANNEL);
    ledcAttachPin(PWMB, RIGHT_PWM_CHANNEL);

    disableMotors();

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENC_A),
        leftEncoderISR,
        RISING);
    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENC_A),
        rightEncoderISR,
        RISING);

    bool initialButtonState =
        digitalRead(BOOT_BUTTON);
    lastButtonReading = initialButtonState;
    stableButtonState = initialButtonState;
    buttonArmed = (initialButtonState == HIGH);
    lastDebounceMs = millis();

    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("================================");
    Serial.println("AUTONOMOUS L ROUTE READY");
    Serial.println("1: FORWARD 30CM (520)");
    Serial.println("2: CLOCKWISE 90 DEG (176)");
    Serial.println("3: FORWARD 30CM (520)");
    Serial.println("MOTORS STOPPED");
    Serial.println("PRESS BOOT ONCE TO START");
    Serial.println("================================");
}

void loop()
{
    if (bootButtonPressed())
    {
        if (routeState == ROUTE_READY)
            beginRouteCountdown();
        else
            abortRoute("USER STOP");
    }

    switch (routeState)
    {
        case ROUTE_READY:
            break;

        case ROUTE_COUNTDOWN:
            if ((millis() - countdownStartMs) >=
                COUNTDOWN_MS)
            {
                Serial.println("SEGMENT 1: FORWARD 30CM");
                routeState = ROUTE_FORWARD_1;
                startMotion(
                    MOTION_FORWARD,
                    FORWARD_30CM_COUNT);
            }
            break;

        case ROUTE_FORWARD_1:
        {
            MotionResult result = updateMotion();
            if (result == MOTION_COMPLETE)
            {
                routeState = ROUTE_PAUSE_1;
                pauseStartMs = millis();
                Serial.println("PAUSE 1 SECOND");
            }
            break;
        }

        case ROUTE_PAUSE_1:
            if ((millis() - pauseStartMs) >= PAUSE_MS)
            {
                Serial.println("SEGMENT 2: CLOCKWISE 90");
                routeState = ROUTE_TURN;
                startMotion(
                    MOTION_CLOCKWISE,
                    CLOCKWISE_90_COUNT);
            }
            break;

        case ROUTE_TURN:
        {
            MotionResult result = updateMotion();
            if (result == MOTION_COMPLETE)
            {
                routeState = ROUTE_PAUSE_2;
                pauseStartMs = millis();
                Serial.println("PAUSE 1 SECOND");
            }
            break;
        }

        case ROUTE_PAUSE_2:
            if ((millis() - pauseStartMs) >= PAUSE_MS)
            {
                Serial.println("SEGMENT 3: FORWARD 30CM");
                routeState = ROUTE_FORWARD_2;
                startMotion(
                    MOTION_FORWARD,
                    FORWARD_30CM_COUNT);
            }
            break;

        case ROUTE_FORWARD_2:
        {
            MotionResult result = updateMotion();
            if (result == MOTION_COMPLETE)
                finishRoute();
            break;
        }
    }

    delay(2);
}
