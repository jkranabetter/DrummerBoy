#include <Metro.h>
#include <AltSoftSerial.h>    // Arduino build environment requires this
#include <wavTrigger.h>
#include "Adafruit_LiquidCrystal.h"
#include <Wire.h>   //EEPROM 24LC256 pin 5 to SDA(20), 24LC256 pin 5 to SCL(21)
#include <EEPROMAnythingEXTERNAL.h> // for extrnal memory ( 24LC256 ) saving/loading

Metro seqMetro(1000);        // Sequencer state machine interval timer
Metro tripletMetro(1000);
Metro readAnalogInputs(100);      // Read potentiometer values
Metro readDigitalInputsHP(50);     //
Metro readDigitalInputsLP(150);

Adafruit_LiquidCrystal lcd(51, 50, 49, 48, 47, 46); // define the pins used to connect to the LCD display

//WAV Trigger will only play WAV files formatted as 16-bit, stereo, 44.1kHz, and there can be no meta-data (non-audio data) in the file before the audio data.
//Make sure the file name starts with a 3 digit number.
wavTrigger wTrig;             // Our WAV Trigger object


// ------------------------------------------------------------------    DECLARE PINS AND CONSTANTS ------------------------------------------------------------//
// const avoids the pitfalls of the define operator (essentially search and replace)

#define deviceaddress 0x50    //Address of 24LC256 eeprom chip
#define NUM_TRACKS 128

#define REDLITE 4
#define GREENLITE 5
#define BLUELITE 6

const int brightness = 100; // you can change the overall brightness by range 0 -> 255

const int sampleRatePin = A0; // Analog input pins
const int swingPin = A1;
const int trckVolPn = A2;
const int tempoPin = A3;
const int volumePin = A4;
const int groupPin = A5;

const int stutterSwitchPin = 19;
const int trackVolBtnPin = 18;
const int swingSwitchPin = 17;
const int pitchSwitchPin = 16;
const int pauseSwitchPin = 43;

const int gSwitchPin[] = {A8, A9, A10, A11, A12, A13, A14, A15}; // define group switch pins

const int sampleTogglePin = 13;
const int functionBtnPin = 40;
const int tapBtnPin = 41;
const int polyBtnPin = 42;
const int eraseBtnPin = 44;
const int accentSelectPin = 45;
const int clockOutPin = 8;

// the 16 beat buttons are connected to digital pins 22 - 37
const int buttonPin[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37}; // define button pins

//--------- serial control for the 24 leds through 3 74HC595 chips
const int dataPin = 10;  ////Pin connected to DS of 74HC595 (Purple)
const int latchPin = 11; //Pin connected to ST_CP of 74HC595 (GREEN)
const int clockPin = 12; //Pin connected to SH_CP of 74HC595 (Yellow)

//--------------------------------------------------------------VARIABLE DECLARATION & INITIALIZATION------------------------------------------------------//

// arrays containing sample wav numbers
// since we use byte for storage the sample numbers are limited to 0-255 range, this is easily changed but uses more dynamic memory (sram)
const byte hat[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
const byte snare[] = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
const byte kick[] = {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48};
const byte perc1[] = {49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};
const byte perc2[] = {65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80};
const byte sample1[] = {81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96};
const byte sample2[] = {97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112};
const byte hit[] = {113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128};
const byte livePlaySamples[16] = {250, 251, 11, 15, 21, 27, 31, 33, 35, 37, 38, 43, 44, 77, 78, 79};

// variables for selecting sample group
boolean hatGroupSelect = true;
boolean snareGroupSelect = false;
boolean kickGroupSelect = false;
boolean perc1GroupSelect = false;
boolean perc2GroupSelect = false;
boolean sample1GroupSelect = false;
boolean sample2GroupSelect = false;
boolean hitGroupSelect = false;

byte groupLedData;
// for displaying selection
boolean buttonsActive[16];
boolean accentBtnsActive[16];
boolean rollBtnsActive[16];
//more array variables
byte chain[4] = {1, 0, 0, 0};
byte playSamplesBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte playSamplesDelayBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte playSamplesTripletDelayBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte selectionMem[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //holds the selected sample for each group
boolean lastAccented[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // 1 element per group - flag is set when accent is activated on the next beat and deactivated when the next beat is not accented
char trackVol[8] = {0, 0, 0, 0, 0, 0, 0, 0};     // char is signed 8 bit (2's compliment so -128 to 127)
byte lastSmpl_Accntd[8] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean groupSwitchState[8] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean delayGroup[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char savedPatternNames[16][16];


//bool & ints
boolean clockOutLogicHigh = false;
boolean beatBtnPshdRN = 0;
boolean accentBtnPshdRN = 0;
boolean polyBtnPshdRN = 0;
boolean rollBtnPshdRN = 0;
boolean groupSelectPushedRN = 0;
boolean tapBtnPushedRN = 0;
boolean sampleSelectPushedRN = 0;
boolean erasePushedRN = 0;
boolean accentPushedRN = 0;
boolean functionBtnPushedRN = 0;

boolean trackVolBtnPushedRN = 0;
boolean polyBtnPushedRN = 0;
boolean accent = 0;
boolean poly = 0;
word currentGroup = 0;
word lastGroup = 1;
word currentSlotPtr = 0;
word tempStorage = 0;
word lastBtnPushed;
word lastAccentBtnPushed;
word lastRollBtnPushed;
word lastPolyBtnPushed;
//boolean patternA = true; // first pattern active - if low pattern B is active (chorus/verse)
boolean polyFlag = false;
boolean tripletFlag = false;
boolean allTripletFlag = false;
boolean copyPatternFlag = false;
boolean fillFlag = false;
boolean restoreChainFlag = false;
boolean liveMash = false;   //when active live mode plays samples rapidly while btn is pushed, when off live mode behaves normally
boolean lockedSamples = true;
boolean pitchOn = false;
word numPatternsSelected = 0;
word copyPtr = 0;
word copyFrom = 0;
word copyTo = 0;
byte selectedSampleNumber = 0;
boolean newSampleSelected = false;  //indicates that a beat button has been played while pressing down the sample button (sample preview)
boolean erasePerformed = false;
static int tempoMS;
static int tripletMS;
static int volume;
word beatMaster = 0;
byte beatMasterTriplet = 0;
word beatHat = 0;  // set initial beat locations
word beatSnare = 0;
word beatKick = 0;
word beatPerc1 = 0;
word beatPerc2 = 0;
word beatSmp1 = 0;
word beatHit = 0;
word beatSmp2 = 0;
word prevBeatHat = 15;
word prevBeatSnare  = 15;
word prevBeatKick = 15;
word prevBeatPerc1 = 15;
word prevBeatPerc2 = 15;
word prevBeatSmp1 = 15;
word prevBeatHit = 15;
word prevBeatSmp2 = 15;
word lastHatBeat = 16;
word lastSnareBeat = 16;
word lastKickBeat = 16;
word lastBassBeat = 16;
word lastPerc1Beat = 16;
word lastPerc2Beat = 16;
word lastSmp1Beat = 16;
word lastHitBeat = 16;
word lastSmp2Beat = 16;
word selectedLastBeat = 0;
word accentDBOffset = 20;      // default 20
int startTime;   // for "tap" entry timing
int startTimeTriplet;
long lastNormalRollMs = 0;
long lastTripletRollMs = 0;
long lastClockMs = 0;
word rollCounterNormal = 0;
word rollCounterTriplet = 0;
int reading;
int halfMS;
int halfTripletMS;
//led display variables
byte data;
byte data2;
byte segmentData[10];
boolean groupLeds[8];
boolean displayingLiveMash = false;
int tempoBPM;
char saveName[16];
int accentVolLastIntermediate;

boolean swingOn = false;
boolean stutterOn = false;
boolean pauseOn = false;
int swingMS = 0;
boolean synced = false;
boolean fastSynced = false;

char trackVolumes[NUM_TRACKS] = {0}; // char data type range is -128 to 127 which is perfect for out volume range of -70 to 10

int delayTime = 40;
boolean delayFlag = 0;
boolean delayTripletFlag = 0;
long startMS_d = 0;
long startMS_tripD = 0;
word lastObj = 0;
word lastPatt = 0; // holds last pattern that was printed to lcd

boolean erasingHats = false;
boolean erasingSnares = false;
boolean erasingKicks = false;
boolean erasingPerc1 = false;
boolean erasingPerc2 = false;
boolean erasingSmp1 = false;
boolean erasingSmp2 = false;
boolean erasingHits = false;
boolean erasingAll = false;


//----------------------------------------------------------------------------------SAMPLE NAMES----------------------------------------------------------------------------------------------//
// save sample names in flash memory in order to preserve precious precious sram

const char sampleName0[] PROGMEM = "808 closed hat";  // Hats
const char sampleName1[] PROGMEM = "elec hat";
const char sampleName2[] PROGMEM = "mouse click";
const char sampleName3[] PROGMEM = "ice hat";
const char sampleName4[] PROGMEM = "jungle hat";
const char sampleName5[] PROGMEM = "open hat 1";
const char sampleName6[] PROGMEM = "open hat 2";
const char sampleName7[] PROGMEM = "open hat 3";
const char sampleName8[] PROGMEM = "click";
const char sampleName9[] PROGMEM = "closed hat";
const char sampleName10[] PROGMEM = "open hat 4";
const char sampleName11[] PROGMEM = "big crash";
const char sampleName12[] PROGMEM = "tiny hat";
const char sampleName13[] PROGMEM = "splash f";
const char sampleName14[] PROGMEM = "chiptune o hat";
const char sampleName15[] PROGMEM = "chiptune c hat";
const char sampleName16[] PROGMEM = "808 Snare";    // Snares
const char sampleName17[] PROGMEM = "collider snare";
const char sampleName18[] PROGMEM = "echo snap";
const char sampleName19[] PROGMEM = "hip snap";
const char sampleName20[] PROGMEM = "clap";
const char sampleName21[] PROGMEM = "echo snare";
const char sampleName22[] PROGMEM = "elec snare";
const char sampleName23[] PROGMEM = "Empty";
const char sampleName24[] PROGMEM = "FPC Rim";
const char sampleName25[] PROGMEM = "cassette Snr";
const char sampleName26[] PROGMEM = "snare hi 1";
const char sampleName27[] PROGMEM = "snare hi 2";
const char sampleName28[] PROGMEM = "chiptune snare 1";
const char sampleName29[] PROGMEM = "chiptune snare 2";
const char sampleName30[] PROGMEM = "chiptune snare 3";
const char sampleName31[] PROGMEM = "empty";
const char sampleName32[] PROGMEM = "808 kick";   // Kicks
const char sampleName33[] PROGMEM = "DNC kick";
const char sampleName34[] PROGMEM = "MAL kick";
const char sampleName35[] PROGMEM = "WhoDat kick";
const char sampleName36[] PROGMEM = "Linn Kick";
const char sampleName37[] PROGMEM = "House kick";
const char sampleName38[] PROGMEM = "frame drum";
const char sampleName39[] PROGMEM = "house gen";
const char sampleName40[] PROGMEM = "brick 808";
const char sampleName41[] PROGMEM = "ms20";
const char sampleName42[] PROGMEM = "808 Drop";
const char sampleName43[] PROGMEM = "chiptune kick";
const char sampleName44[] PROGMEM = "empty";
const char sampleName45[] PROGMEM = "empty";
const char sampleName46[] PROGMEM = "empty";
const char sampleName47[] PROGMEM = "empty";
const char sampleName48[] PROGMEM = "Guiro";  // Perc 1
const char sampleName49[] PROGMEM = "bell shake";
const char sampleName50[] PROGMEM = "808 cowbell";
const char sampleName51[] PROGMEM = "Chika Crunch";
const char sampleName52[] PROGMEM = "Crunch 1";
const char sampleName53[] PROGMEM = "Bubble 1";
const char sampleName54[] PROGMEM = "Bubble 2";
const char sampleName55[] PROGMEM = "Bubble 3";
const char sampleName56[] PROGMEM = "Bloop";
const char sampleName57[] PROGMEM = "salt rim";
const char sampleName58[] PROGMEM = "glassbell";
const char sampleName59[] PROGMEM = "Chika Perc";
const char sampleName60[] PROGMEM = "Drip";
const char sampleName61[] PROGMEM = "soda";
const char sampleName62[] PROGMEM = "clav reverb";
const char sampleName63[] PROGMEM = "909 clav";
const char sampleName64[] PROGMEM = "pop";      // Perc 2
const char sampleName65[] PROGMEM = "stapler";
const char sampleName66[] PROGMEM = "phone 1";
const char sampleName67[] PROGMEM = "phone 2";
const char sampleName68[] PROGMEM = "phone 3";
const char sampleName69[] PROGMEM = "egg shaker";
const char sampleName70[] PROGMEM = "seed shaker";
const char sampleName71[] PROGMEM = "shutter 1";
const char sampleName72[] PROGMEM = "shutter 2";
const char sampleName73[] PROGMEM = "chiptune tom 1";
const char sampleName74[] PROGMEM = "chiptune tom 2";
const char sampleName75[] PROGMEM = "chiptune tom 3";
const char sampleName76[] PROGMEM = "empty";
const char sampleName77[] PROGMEM = "empty";
const char sampleName78[] PROGMEM = "empty";
const char sampleName79[] PROGMEM = "empty";
const char sampleName80[] PROGMEM = "cha!";  //Samples 1
const char sampleName81[] PROGMEM = "game points";
const char sampleName82[] PROGMEM = "Nova beep";
const char sampleName83[] PROGMEM = "droplet";
const char sampleName84[] PROGMEM = "pong";
const char sampleName85[] PROGMEM = "echo harp";
const char sampleName86[] PROGMEM = "punch";
const char sampleName87[] PROGMEM = "chika A";
const char sampleName88[] PROGMEM = "chika B";
const char sampleName89[] PROGMEM = "chika C";
const char sampleName90[] PROGMEM = "chicka D";
const char sampleName91[] PROGMEM = "chika E";
const char sampleName92[] PROGMEM = "wow";
const char sampleName93[] PROGMEM = "Laser";
const char sampleName94[] PROGMEM = "noisefall";
const char sampleName95[] PROGMEM = "letmehearUsay";
const char sampleName96[] PROGMEM = "crowd chant"; // Samples 2
const char sampleName97[] PROGMEM = "im a gun man";
const char sampleName98[] PROGMEM = "sound of wealth";
const char sampleName99[] PROGMEM = "i understand now";
const char sampleName100[] PROGMEM = "going up";
const char sampleName101[] PROGMEM = "empty";
const char sampleName102[] PROGMEM = "empty";
const char sampleName103[] PROGMEM = "empty";
const char sampleName104[] PROGMEM = "empty";
const char sampleName105[] PROGMEM = "empty";
const char sampleName106[] PROGMEM = "empty";
const char sampleName107[] PROGMEM = "empty";
const char sampleName108[] PROGMEM = "empty";
const char sampleName109[] PROGMEM = "c bell";
const char sampleName110[] PROGMEM = "e bell";
const char sampleName111[] PROGMEM = "g bell";
const char sampleName112[] PROGMEM = "orchestral hit 1"; // HITS
const char sampleName113[] PROGMEM = "nitrodeluxe G";
const char sampleName114[] PROGMEM = "nitrodeluxe C";
const char sampleName115[] PROGMEM = "dancemachine C";
const char sampleName116[] PROGMEM = "dancemachine C#";
const char sampleName117[] PROGMEM = "yeah!";
const char sampleName118[] PROGMEM = "sword";
const char sampleName119[] PROGMEM = "getready";
const char sampleName120[] PROGMEM = "churh bell";
const char sampleName121[] PROGMEM = "orchestral hit 2";
const char sampleName122[] PROGMEM = "empty";
const char sampleName123[] PROGMEM = "empty";
const char sampleName124[] PROGMEM = "empty";
const char sampleName125[] PROGMEM = "empty";
const char sampleName126[] PROGMEM = "empty";
const char sampleName127[] PROGMEM = "emptylast";

const char * const string_table[] PROGMEM =     // change "string_table" name to suit
{
  sampleName0,
  sampleName1,
  sampleName2,
  sampleName3,
  sampleName4,
  sampleName5,
  sampleName6,
  sampleName7,
  sampleName8,
  sampleName9,
  sampleName10,
  sampleName11,
  sampleName12,
  sampleName13,
  sampleName14,
  sampleName15,
  sampleName16,
  sampleName17,
  sampleName18,
  sampleName19,
  sampleName20,
  sampleName21,
  sampleName22,
  sampleName23,
  sampleName24,
  sampleName25,
  sampleName26,
  sampleName27,
  sampleName28,
  sampleName29,
  sampleName30,
  sampleName31,
  sampleName32,
  sampleName33,
  sampleName34,
  sampleName35,
  sampleName36,
  sampleName37,
  sampleName38,
  sampleName39,
  sampleName40,
  sampleName41,
  sampleName42,
  sampleName43,
  sampleName44,
  sampleName45,
  sampleName46,
  sampleName47,
  sampleName48,
  sampleName49,
  sampleName50,
  sampleName51,
  sampleName52,
  sampleName53,
  sampleName54,
  sampleName55,
  sampleName56,
  sampleName57,
  sampleName58,
  sampleName59,
  sampleName60,
  sampleName61,
  sampleName62,
  sampleName63,
  sampleName64,
  sampleName65,
  sampleName66,
  sampleName67,
  sampleName68,
  sampleName69,
  sampleName70,
  sampleName71,
  sampleName72,
  sampleName73,
  sampleName74,
  sampleName75,
  sampleName76,
  sampleName77,
  sampleName78,
  sampleName79,
  sampleName80,
  sampleName81,
  sampleName82,
  sampleName83,
  sampleName84,
  sampleName85,
  sampleName86,
  sampleName87,
  sampleName88,
  sampleName89,
  sampleName90,
  sampleName91,
  sampleName92,
  sampleName93,
  sampleName94,
  sampleName95,
  sampleName96,
  sampleName97,
  sampleName98,
  sampleName99,
  sampleName100,
  sampleName101,
  sampleName102,
  sampleName103,
  sampleName104,
  sampleName105,
  sampleName106,
  sampleName107,
  sampleName108,
  sampleName109,
  sampleName110,
  sampleName111,
  sampleName112,
  sampleName113,
  sampleName114,
  sampleName115,
  sampleName116,
  sampleName117,
  sampleName118,
  sampleName119,
  sampleName120,
  sampleName121,
  sampleName122,
  sampleName123,
  sampleName124,
  sampleName125,
  sampleName126,
  sampleName127
};
char buffer[16];    // make sure this is large enough for the largest string it must hold


//----------------------------------------------------------------------------------OBJECT CREATION-----------------------------------------------------------------------------//
// this class contains all the information contained in 1 Drummer Boy bar of music excluding sample volumes which are only stored/restored when saving/loading

class Bar
{
  public:
    byte hatActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte snareActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte kickActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte perc1ActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte perc2ActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte smp1ActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte smp2ActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    byte hitActiveSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    byte rollSamples[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    boolean tripletGroup[8] = {false,  false, false, false, false, false, false, false };
    boolean shuffleGroup[8] = {false,  false, false, false, false, false, false, false };

    word hatAccents;
    word snareAccents;
    word kickAccents;
    word perc1Accents;
    word perc2Accents;
    word smpAccents;
    word smp2Accents;
    word hitAccents;

    word buttonsHat;
    word buttonsSnare;
    word buttonsKick;
    word buttonsPerc1;
    word buttonsPerc2;
    word buttonsSmp1;
    word buttonsSmp2;
    word buttonsHit;

};

// 192 bytes each
Bar a;
Bar b;
Bar c;
Bar f;

struct saveLoad     // declare a structure for saving/loading
{
  Bar a_ROM;
  Bar b_ROM;
  Bar c_ROM;
  Bar f_ROM;
  char trackVolumes_ROM[NUM_TRACKS] = {0};
  byte chain_ROM[4] = {0};  //modifymem
} memory;

//------------------------------------------------------------------- FUNCTION PROTOTYPES--------------------------------------------------------------------------------------------------//

// FUNCTION PROTOTYPES // -  The Arduino IDE creates function prototypes for you. Normally, this works quite well. There are some situations, like functions with reference arguments, where it doesn't.
void setAccents(Bar &obj, boolean triplet);
void setAccent(int i, int nextBeat, word accentArray, Bar &obj);
void checkAnalogIO(Bar &obj);
void loadNextBeat(Bar &obj, boolean triplet);
void checkAccentButtons(Bar &obj);
void checkDigitalIO(Bar &obj);
void checkMedPriorityButtons(Bar &obj);
void save(Bar &obj);
void load(Bar &obj);
void checkLowPriorityButtons(Bar &obj);
void changeActiveSamples(int last, int current, Bar &obj);
void copyToActiveGroup(Bar &obj);
void copyTFromActiveGroup(Bar &obj);
void addSample(int beat, Bar &obj);
void checkBeatButtons(Bar &obj);
void checkHighPriorityButtons(Bar &obj);
void switchGroup(Bar &obj);
void removeSample(int beat, Bar &obj);
void readTrackVolume(boolean firstCall, Bar &obj);
void readGroup(boolean firstCall, Bar &obj);
void setTrackVolumes();
void readBtn_Sample(Bar &obj);
void readBtn_Erase(Bar &obj);
void readBtn_Accent(Bar &obj);
void readBtn_TrackVol(Bar &obj);
void readBtn_Poly(Bar &obj);
void readBtn_Tap(Bar &obj);
void readBtn_Function(Bar &obj);
int getSelection(Bar & obj, boolean saving);
void playRoll(Bar &obj);
void addRollSample(int beat, Bar &obj);
void removeRollSample(int beat, Bar &obj);
void incrementBeatCounters(boolean triplet, Bar &obj);
void setTriplet(Bar &obj);
boolean tripletActiveforCrntGrp(Bar &obj);
void waitForDelay(Bar &obj);
void resetBeat(Bar &obj);
void checkPattern(Bar &obj);
Bar getObjectByRef(int ptr);
void mainLoop(Bar &object);
Bar getOb();
void randomize(Bar &obj);
void randomizeContinuously(Bar &obj);
void shuffle(Bar &obj);
void stutter(Bar &obj);
void playNormalRoll(Bar &obj, byte beat);
void playTripletRoll(Bar &obj, byte beat);
//void updateLCD(boolean updateSampleName, int updateBpm, int);



// ***************************************SETUP*******************************************************
void setup() {
  // set default track volumes to prevent anvil/stirrup combustion
  trackVolumes[4] = -4; //ice hat
  trackVolumes[9] = -4; //click
  trackVolumes[11] = -4; //open hat
  trackVolumes[12] = -20; //grav crash
  trackVolumes[15] = -10; //chiptune hh open
  trackVolumes[16] = -5; //chiptune hh closed
  trackVolumes[29] = -5; //chiptune snare 1
  trackVolumes[30] = -5; //chiptune snare 2
  trackVolumes[31] = -5; //chiptune snare 3
  trackVolumes[33] = -5; //808 kick
  trackVolumes[35] = -8; //MAL kick
  trackVolumes[37] = -8; //LINN kick
  trackVolumes[38] = -10; //House kick
  trackVolumes[40] = -10; //House gen
  trackVolumes[57] = -5; //bloop
  trackVolumes[58] = -15; //triangle
  trackVolumes[59] = -15; //glassbell
  trackVolumes[64] = -15; //909 clav
  trackVolumes[67] = -15; //phone 1
  trackVolumes[67] = -5; //phone 3
  trackVolumes[81] = -10; //cha1
  trackVolumes[86] = -15; //echo harp
  trackVolumes[87] = -10; //punch
  trackVolumes[88] = -10; //chicka 1
  trackVolumes[94] = -15; //laser
  trackVolumes[95] = -10; //noisefall
  trackVolumes[96] = -15; //let me hear you say
  trackVolumes[98] = -5; //gun
  trackVolumes[99] = -20; //sound of wealth
  trackVolumes[100] = -5; //oh
  trackVolumes[101] = -10; //going up
  trackVolumes[113] = -12; //orchestral hit
  trackVolumes[116] = -12; //dancemachine
  trackVolumes[117] = -12; //dancemachine
  trackVolumes[118] = -5; //yea
  trackVolumes[119] = -12; //sword
  trackVolumes[120] = -12; //get ready
  trackVolumes[121] = -12; //church bell

  // create objects, each is 1 bar (16 beats)
  a = Bar();
  b = Bar();
  c = Bar();
  f = Bar();

  //for eeprom SDA SCL (I2C communication)
  Wire.begin();
  // Serial monitor for debugging
  Serial.begin(9600);

  pinMode(latchPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockOutPin, OUTPUT);

  segmentData[0] = 0b10000001; //0
  segmentData[1] = 0b11110011; //1
  segmentData[2] = 0b01001001; //2
  segmentData[3] = 0b01100001; //3
  segmentData[4] = 0b00110011; //4
  segmentData[5] = 0b00100101; //5
  segmentData[6] = 0b00000101; //6
  segmentData[7] = 0b11110001; //7
  segmentData[8] = 0b00000001; //8
  segmentData[9] = 0b00100001; //9

  pinMode(buttonPin[0], INPUT_PULLUP);
  pinMode(buttonPin[1], INPUT_PULLUP);
  pinMode(buttonPin[2], INPUT_PULLUP);
  pinMode(buttonPin[3], INPUT_PULLUP);
  pinMode(buttonPin[4], INPUT_PULLUP);
  pinMode(buttonPin[5], INPUT_PULLUP);
  pinMode(buttonPin[6], INPUT_PULLUP);
  pinMode(buttonPin[7], INPUT_PULLUP);
  pinMode(buttonPin[8], INPUT_PULLUP);
  pinMode(buttonPin[9], INPUT_PULLUP);
  pinMode(buttonPin[10], INPUT_PULLUP);
  pinMode(buttonPin[11], INPUT_PULLUP);
  pinMode(buttonPin[12], INPUT_PULLUP);
  pinMode(buttonPin[13], INPUT_PULLUP);
  pinMode(buttonPin[14], INPUT_PULLUP);
  pinMode(buttonPin[15], INPUT_PULLUP);
  pinMode(eraseBtnPin, INPUT_PULLUP);
  pinMode(functionBtnPin, INPUT_PULLUP);
  pinMode(accentSelectPin, INPUT_PULLUP);
  pinMode(polyBtnPin, INPUT_PULLUP);
  pinMode(tapBtnPin, INPUT_PULLUP);
  pinMode(trackVolBtnPin, INPUT_PULLUP);

  pinMode(sampleTogglePin, INPUT_PULLUP);

  pinMode(pitchSwitchPin, INPUT_PULLUP);
  pinMode(swingSwitchPin, INPUT_PULLUP);
  pinMode(stutterSwitchPin, INPUT_PULLUP);
  pinMode(pauseSwitchPin, INPUT_PULLUP);
  pinMode(gSwitchPin[0], INPUT_PULLUP);
  pinMode(gSwitchPin[1], INPUT_PULLUP);
  pinMode(gSwitchPin[2], INPUT_PULLUP);
  pinMode(gSwitchPin[3], INPUT_PULLUP);
  pinMode(gSwitchPin[4], INPUT_PULLUP);
  pinMode(gSwitchPin[5], INPUT_PULLUP);
  pinMode(gSwitchPin[6], INPUT_PULLUP);
  pinMode(gSwitchPin[7], INPUT_PULLUP);

  delay(500); // wait for the WAV Trigger to finish reset before trying to send commands.
  wTrig.start(); // WAV Trigger startup at 57600
  delay(10);
  wTrig.stopAllTracks();// Send a stop-all command and reset the sample-rate offset, in case we have reset while the WAV Trigger was already playing.
  wTrig.samplerateOffset(0); // make sure the pitch is unmodified


  // enable track reporting (recieve data from the WAV Trigger)
  wTrig.setReporting(true);
  delay(100);
  // get version/track info from the wav trigger
  char gWTrigVersion[VERSION_STRING_LEN];    // WAV Trigger version string
  if (wTrig.getVersion(gWTrigVersion, VERSION_STRING_LEN)) {
    Serial.print(gWTrigVersion);
    Serial.print("\n");
    int gNumTracks = wTrig.getNumTracks();
    Serial.print("Number of tracks = ");
    Serial.println(gNumTracks);
  }
  else
    Serial.print("WAV Trigger response not available");

  updateGroupData();
  formatTrackVolumes(); // reset all track volumes to 0
  setTrackVolumes();
  tripletMetro.previous_millis = seqMetro.previous_millis;    //sync timers

  // setup lcd
  lcd.begin(16, 2);
  lcd.print("Drummer Boy!");
  lcd.setCursor(0, 1);
  lcd.write(byte(0));
  //setBacklight(100, 0, 255 - 100);  //set backlight color
  delay(1000);
  updateLCD(true, 0, true);  // update all info on the lcd

  readTempo(1); // read vol and tempo values
  readVolume(1);
  wTrig.trackPlayPoly(200);   // play startup sound

  EEPROM_readAnything(900 * 16, savedPatternNames);

  //  for (int i = 0 ; i < 16; i++) {   // format save names
  //    savedPatternNames[i][0] = 'e';
  //    savedPatternNames[i][1] = 'm';
  //    savedPatternNames[i][2] = 'p';
  //    savedPatternNames[i][3] = 't';
  //    savedPatternNames[i][4] = 'y';
  //    savedPatternNames[i][5] = ' ';
  //    savedPatternNames[i][6] = ' ';
  //    savedPatternNames[i][7] = ' ';
  //    savedPatternNames[i][8] = ' ';
  //    savedPatternNames[i][9] = ' ';
  //    savedPatternNames[i][10] = ' ';
  //    savedPatternNames[i][11] = ' ';
  //    savedPatternNames[i][12] = ' ';
  //    savedPatternNames[i][13] = ' ';
  //    savedPatternNames[i][14] = ' ';
  //    savedPatternNames[i][15] = ' ';
  //  }

}

//------------------------------------------------------------------------------------MAIN LOOP------------------------------------------------------------------------//


void loop() {
  if (chain[currentSlotPtr] == 1) mainLoop(a);
  if (chain[currentSlotPtr] == 2) mainLoop(b);
  if (chain[currentSlotPtr] == 3) mainLoop(c);
  if (chain[currentSlotPtr] == 4) mainLoop(f);
}


void mainLoop(Bar &object) {
  if (lastObj != chain[currentSlotPtr]) {
    copyToActiveGroup(object);
    lastObj = chain[currentSlotPtr];
  }
  if (!pauseOn) {
    if (seqMetro.check() == 1) {
      //Serial.println("quarter");
      setSwing();
      setAccents(object, false); //set accents before to prevent change in volume while track is playing
      lastNormalRollMs = millis(); // for roll
      rollCounterNormal = 0; // for roll
      for (int i = 0; i < 8 ; i++) {
        if ( playSamplesBuffer[i] != 0) {
          if (groupSwitchState[i] == 1 && !object.tripletGroup[i]) {
            if (!delayGroup[i]) {
              wTrig.trackPlayPoly( playSamplesBuffer[i] );
              //Serial.print("Playing sample:"); Serial.println(playSamplesBuffer[i]);
            }
            else {
              playSamplesDelayBuffer[i] = playSamplesBuffer[i];
              startMS_d = millis();
              delayFlag = true;
              Serial.print("liveMash:"); Serial.println(liveMash);
            }
          }
        }
      }
      if (delayFlag == false) {
        incrementBeatCounters(false, object);
        if (beatMaster == 14)shuffle(object);
        if (beatMaster == 15) checkPattern(object);
        loadNextBeat(object, false);
      }
      startTime = millis();
    }
    if (tripletMetro.check() == 1) {
      //Serial.println("triplet");
      setAccents(object, true); //set accents before to prevent change in volume while track is playing
      lastTripletRollMs = millis(); // for roll
      rollCounterTriplet = 0; // for roll
      for (int i = 0; i < 8 ; i++) {
        if ( playSamplesBuffer[i] != 0) {
          if (groupSwitchState[i] == 1 && object.tripletGroup[i]) {
            if (!delayGroup[i]) {
              wTrig.trackPlayPoly( playSamplesBuffer[i] );
              Serial.println("Normal Triplet");
            }
            else {
              playSamplesTripletDelayBuffer[i] = playSamplesBuffer[i];
              startMS_tripD = millis();
              delayTripletFlag = true;
              Serial.print("1T:"); Serial.println(millis());
            }
          }
        }
      }
      if (delayTripletFlag == false) {
        incrementBeatCounters(true, object);
        loadNextBeat(object, true);
      }
      startTime = millis();
    }
    waitForDelay(object);
    playRoll(object);
    checkDigitalIO(object);
    checkAnalogIO(object);
    clockOut();
    if (stutterOn) stutter(object);
    updateLCD(false, 0, false);
  } else {
    if (digitalRead(pauseSwitchPin) == HIGH && pauseOn) {
      pauseOn = false;
      seqMetro.reset();
      tripletMetro.reset();
      startAtBeat0();
    }
    checkDigitalIO(object);
    checkAnalogIO(object);
  }
}
////////////////////////////////////////////////////////////////////////////////////// OOP Methods /////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void clockOut() {
  if (millis() - lastClockMs > tempoMS / 32) {
    if (clockOutLogicHigh) {
      digitalWrite(clockOutPin, LOW);
      clockOutLogicHigh = false;
    }
    else {
      digitalWrite(clockOutPin, HIGH);
      clockOutLogicHigh = true;
    }
    lastClockMs = millis();
  }
}

void stutter(Bar &obj) {
  if ( !obj.tripletGroup[0]) beatHat = 15;
  else beatHat = 14;
  if ( !obj.tripletGroup[1]) beatSnare = 15;
  else beatSnare = 14;
  if ( !obj.tripletGroup[2]) beatKick = 15;
  else beatKick = 14;
  if ( !obj.tripletGroup[3]) beatPerc1 = 15;
  else beatPerc1 = 14;
  if ( !obj.tripletGroup[4]) beatPerc2 = 15;
  else beatPerc2 = 14;
  if ( !obj.tripletGroup[5]) beatSmp1 = 15;
  else beatSmp1 = 14;
  if ( !obj.tripletGroup[6]) beatSmp2 = 15;
  else beatSmp2 = 14;
  if ( !obj.tripletGroup[7]) beatHit = 15;
  else beatHit = 14;
}

void updateLCD(boolean updateSampleName, int updateBpm, boolean updatePattern) {
  if(accent){
    lcd.home();
    lcd.print("accent decibel  ");
    lcd.setCursor(0, 1);
    lcd.print("difference: ");
    lcd.print(accentDBOffset);
    lcd.print("   ");
    return;
  }else if (erasePushedRN) {
    lcd.home();
    lcd.print("select btn below");
    lcd.setCursor(0, 1);
    lcd.print("group to erase!");
    return;
  } else if (liveMash && !displayingLiveMash) {
    displayingLiveMash = true;
    Serial.println("PRINTING LIVE");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("LIVE");
    return;
  }
  if (updateSampleName) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    int currSamp = getCrrntSlctdSmpl();
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[currSamp - 1]))); // Necessary casts and dereferencing, just copy.
    lcd.print(buffer);  //untested!!!
  }
  if (updateBpm > 0) {
    lcd.setCursor(10, 0);
    lcd.print(updateBpm);
    lcd.print("BPM  ");
  }
  if (chain[currentSlotPtr] != lastPatt || updatePattern) {
    lcd.home();
    lcd.print("Pattern:");
    if (chain[currentSlotPtr] == 1) lcd.print("A");
    if (chain[currentSlotPtr] == 2) lcd.print("B");
    if (chain[currentSlotPtr] == 3) lcd.print("C");
    if (chain[currentSlotPtr] == 4) lcd.print("D");
    lastPatt = chain[currentSlotPtr];
    lcd.print(" ");
  }
}

void addPattern(int pattern) {    //set the pattern order
  if (numPatternsSelected < 4) {
    Serial.print("Add pattern:"); Serial.print(pattern); Serial.print(" to slot:"); Serial.println(numPatternsSelected);
    chain[numPatternsSelected] = pattern;
    numPatternsSelected++;
  }
}
void checkPattern(Bar &obj) {   //checked at the end of every bar, plays the next patten in the 'chain'
  copyFromActiveGroup(obj);
  if (restoreChainFlag) { //restores old chain after the fill has played before incrementing the ptr
    chain[currentSlotPtr] = tempStorage;
    restoreChainFlag = false;
  }
  if (chain[1] == 0) currentSlotPtr = 0;      //reset pointer if the next element is zero, otherwise increment it
  else if (chain[2] == 0 && currentSlotPtr == 1) currentSlotPtr = 0;
  else if (chain[3] == 0 && currentSlotPtr == 2) currentSlotPtr = 0;
  else currentSlotPtr++;
  if (currentSlotPtr == 4) currentSlotPtr = 0;  //reset pointer
  if (fillFlag) { // set fill
    tempStorage = chain[currentSlotPtr];
    chain[currentSlotPtr] = 4;
    fillFlag = false; restoreChainFlag = true;
  }
}

void copy(int pat) {
  Serial.print("Copy...");
  if (copyPtr < 2) {  // no need to do anything if a third button is pressed
    if (copyPtr == 0) { // store selection from first push
      copyFrom = pat;
    }
    if (copyPtr == 1) { //second push we copy the selected pattern over
      copyTo = pat;
      if (copyFrom == 1 && copyTo == 2) memcpy(&b, &a, sizeof(a));
      else if (copyFrom == 1 && copyTo == 3) memcpy(&c, &a, sizeof(a));
      else if (copyFrom == 1 && copyTo == 4) memcpy(&f, &a, sizeof(a));
      else if (copyFrom == 2 && copyTo == 1) memcpy(&a, &b, sizeof(a));
      else if (copyFrom == 2 && copyTo == 3) memcpy(&c, &b, sizeof(a));
      else if (copyFrom == 2 && copyTo == 4) memcpy(&f, &b, sizeof(a));
      else if (copyFrom == 3 && copyTo == 1) memcpy(&a, &c, sizeof(a));
      else if (copyFrom == 3 && copyTo == 2) memcpy(&b, &c, sizeof(a));
      else if (copyFrom == 3 && copyTo == 4) memcpy(&f, &c, sizeof(a));
      else if (copyFrom == 4 && copyTo == 1) memcpy(&a, &f, sizeof(a));
      else if (copyFrom == 4 && copyTo == 2) memcpy(&b, &f, sizeof(a));
      else if (copyFrom == 4 && copyTo == 3) memcpy(&c, &f, sizeof(a));
      Serial.print("Copy pattern"); Serial.print(copyFrom); Serial.print(" to "); Serial.println(copyTo);
    }
    copyPtr++;
  }
}

//------------------------------------------------------------------------------------LOOP METHODS------------------------------------------------------------------------//
void waitForDelay(Bar &obj) {
  if (delayFlag)
    for (int i = 0; i < 7 ; i++)
      if (delayGroup[i] && millis() > startMS_d + delayTime) {
        wTrig.trackPlayPoly( playSamplesDelayBuffer[i] );
        playSamplesDelayBuffer[i] = 0;
        incrementBeatCounters(false, obj);
        if (beatMaster == 15) checkPattern(obj);
        loadNextBeat(obj, false);
        delayFlag = false;
        Serial.print("2:"); Serial.println(millis());
      }
  if (delayTripletFlag)
    for (int i = 0; i < 7 ; i++)
      if (delayGroup[i] && millis() > startMS_tripD + delayTime) {
        wTrig.trackPlayPoly( playSamplesTripletDelayBuffer[i] );
        playSamplesTripletDelayBuffer[i] = 0;
        incrementBeatCounters(true, obj);
        loadNextBeat(obj, true);
        delayTripletFlag = false;
        Serial.print("2T:"); Serial.println(millis());
      }
}

void startAtBeat0() {
  beatHat = 0;
  beatSnare = 0;
  beatKick = 0;
  beatPerc1 = 0;
  beatPerc2 = 0;
  beatSmp1 = 0;
  beatSmp2 = 0;
  beatHit = 0;
  beatMaster = 0;
  beatMasterTriplet = 0;

  prevBeatHat = 15;
  prevBeatSnare  = 15;
  prevBeatKick = 15;
  prevBeatPerc1 = 15;
  prevBeatPerc2 = 15;
  prevBeatSmp1  = 15;
  prevBeatSmp2 = 15;
  prevBeatHit = 15;
}

void setDelay() {
  if ( hatGroupSelect ) {
    delayGroup[0] = delayGroup[0] ? false : true;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( snareGroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = delayGroup[1] ? false : true;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( kickGroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = delayGroup[2] ? false : true;
    delayGroup[4] = false;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( perc1GroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = delayGroup[3] ? false : true;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( perc2GroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = delayGroup[4] ? false : true;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( sample1GroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = false;
    delayGroup[5] = delayGroup[5] ? false : true;
    delayGroup[6] = false;
    delayGroup[7] = false;
  }
  else if ( sample2GroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = delayGroup[6] ? false : true;
    delayGroup[7] = false;
  }
  else if (  hitGroupSelect )  {
    delayGroup[0] = false;
    delayGroup[1] = false;
    delayGroup[2] = false;
    delayGroup[3] = false;
    delayGroup[4] = false;
    delayGroup[5] = false;
    delayGroup[6] = false;
    delayGroup[7] = delayGroup[7] ? false : true;
  }
}

void playRoll(Bar & obj) {
  boolean isRollSample = false;
  byte tempBeat;
  byte tempBeat2;
  for ( int i = 0 ; i < 8 ; i++) {  //check for rolls in each group 
    if (!obj.tripletGroup[i]) {   //if the group is 4/4 time
      tempBeat = beatMaster;
      tempBeat = tempBeat == 0 ? 15 : tempBeat - 1;
      isRollSample = obj.rollSamples[tempBeat] != 0;
      if (!isRollSample) continue;      // if
      else playNormalRoll(obj, tempBeat);
    } else {                      // if the group is triplet timing
      if (beatMasterTriplet % 4 == 0) {
        if (beatMasterTriplet == 0 && obj.rollSamples[14] != 0) playTripletRoll(obj, 14);
        else if (beatMasterTriplet == 4 && obj.rollSamples[2] != 0) playTripletRoll(obj, 2);
        else if (beatMasterTriplet == 8 && obj.rollSamples[6] != 0) playTripletRoll(obj, 6);
        else if (beatMasterTriplet == 12 && obj.rollSamples[10] != 0) playTripletRoll(obj, 10);
      } else {
        if (obj.rollSamples[beatMasterTriplet - 1] != 0) playTripletRoll(obj, beatMasterTriplet - 1);
      }
    }
  }
}

void playNormalRoll(Bar & obj, byte beat) {
  if ((millis() - lastNormalRollMs > tempoMS / 4) && rollCounterNormal < 3) {
    if ( obj.rollSamples[beat] == 1 && groupSwitchState[0]) {
      wTrig.trackPlayPoly( obj.hatActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 2 && groupSwitchState[1]) {
      wTrig.trackPlayPoly( obj.snareActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 3 && groupSwitchState[2]) {
      wTrig.trackPlayPoly( obj.kickActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 4 && groupSwitchState[3]) {
      wTrig.trackPlayPoly( obj.perc1ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 5 && groupSwitchState[4]) {
      wTrig.trackPlayPoly( obj.perc2ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 6 && groupSwitchState[5]) {
      wTrig.trackPlayPoly( obj.smp1ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 7 && groupSwitchState[6]) {
      wTrig.trackPlayPoly( obj.smp2ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 8 && groupSwitchState[7]) {
      wTrig.trackPlayPoly( obj.hitActiveSamples[beat] );
    }
    rollCounterNormal++;
    lastNormalRollMs = millis();
  }
}
void playTripletRoll(Bar & obj, byte beat) {
  if ((millis() - lastTripletRollMs > tripletMS / 3) && rollCounterTriplet < 2) {
    if ( obj.rollSamples[beat] == 1 && groupSwitchState[0]) {
      wTrig.trackPlayPoly( obj.hatActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 2 && groupSwitchState[1]) {
      wTrig.trackPlayPoly( obj.snareActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 3 && groupSwitchState[2]) {
      wTrig.trackPlayPoly( obj.kickActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 4 && groupSwitchState[3]) {
      wTrig.trackPlayPoly( obj.perc1ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 5 && groupSwitchState[4]) {
      wTrig.trackPlayPoly( obj.perc2ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 6 && groupSwitchState[5]) {
      wTrig.trackPlayPoly( obj.smp1ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 7 && groupSwitchState[6]) {
      wTrig.trackPlayPoly( obj.smp2ActiveSamples[beat] );
    }
    else if ( obj.rollSamples[beat] == 8 && groupSwitchState[7]) {
      wTrig.trackPlayPoly( obj.hitActiveSamples[beat] );
    }
    rollCounterTriplet++;
    lastTripletRollMs = millis();
  }
}

void setAccents(Bar &obj, boolean triplet) {
  if (triplet == obj.tripletGroup[0])setAccent(0, beatHat, obj.hatAccents, obj);
  if (triplet == obj.tripletGroup[1])setAccent(1, beatSnare, obj.snareAccents, obj);
  if (triplet == obj.tripletGroup[2])setAccent(2, beatKick, obj.kickAccents, obj);
  if (triplet == obj.tripletGroup[3])setAccent(3, beatPerc1, obj.perc1Accents, obj);
  if (triplet == obj.tripletGroup[4])setAccent(4, beatPerc2, obj.perc2Accents, obj);
  if (triplet == obj.tripletGroup[5])setAccent(5, beatSmp1, obj.smpAccents, obj);
  if (triplet == obj.tripletGroup[6])setAccent(6, beatSmp2, obj.smp2Accents, obj);
  if (triplet == obj.tripletGroup[7])setAccent(7, beatHit, obj.hitAccents, obj);
  //Serial.print("Beat: "); Serial.println(beatHat);
}
void setAccent(int i, int nextBeat, word accentArray, Bar & obj) {
  if ( accentArray & 1 << nextBeat && !lastAccented[i]) {
    trackVol[i] =  trackVolumes[ playSamplesBuffer[i] ];    // save the track volume before adjustment so it can be put back after the accent
    lastSmpl_Accntd[i] = playSamplesBuffer[i];    // save the sample number of last accented sample so track vol can be put back after the accent
    int accentVol = trackVol[i] + accentDBOffset;  // generate accent volume (sum of current vol + accent offset)
    if (accentVol > 10 ) accentVol = 10;    //upper limit of 10 dB for wav trigger
    wTrig.trackGain(playSamplesBuffer[i], accentVol);   // set accent volume (higher than regular)
    lastAccented[i] = 1;    // set flag
  }
  else if ( !(accentArray & 1 << nextBeat) && lastAccented[i]) {
    wTrig.trackGain(lastSmpl_Accntd[i], trackVol[i]); // set volume back to the regular track volume after an accent
    lastAccented[i] = 0;
  }
}

boolean isTripletActive(Bar &obj) {
  if (obj.tripletGroup[0] || obj.tripletGroup[1] || obj.tripletGroup[2] || obj.tripletGroup[3] || obj.tripletGroup[4] || obj.tripletGroup[5] || obj.tripletGroup[6] || obj.tripletGroup[7])
    return true;
  else
    return false;
}

void incrementBeatCounters(boolean triplet, Bar &obj) {
  if (!triplet) beatMaster++;
  if (triplet) beatMasterTriplet++;

  if (triplet == obj.tripletGroup[0]) { //increments for triplets and 4/4
    prevBeatHat = beatHat;    // keep track of the previous beat
    beatHat++;
  }
  if (triplet == obj.tripletGroup[1]) {
    prevBeatSnare = beatSnare;
    beatSnare++;
  }
  if (triplet == obj.tripletGroup[2]) {
    prevBeatKick = beatKick;
    beatKick++;
  }
  if (triplet == obj.tripletGroup[3]) {
    prevBeatPerc1 = beatPerc1;
    beatPerc1++;
  }
  if (triplet == obj.tripletGroup[4]) {
    prevBeatPerc2 = beatPerc2;
    beatPerc2++;
  }
  if (triplet == obj.tripletGroup[5]) {
    prevBeatSmp1 = beatSmp1;
    beatSmp1++;
  }
  if (triplet == obj.tripletGroup[6]) {
    prevBeatSmp2 = beatSmp2;
    beatSmp2++;
  }
  if (triplet == obj.tripletGroup[7]) {
    prevBeatHit = beatHit;
    beatHit++;
  }

  // sync the master and triplet beats once at the start of every measure, they are called usually a few ms apart in random order so wait until both have played their first beats
  if (beatMaster == 1 && beatMasterTriplet == 1) {
    if (!synced) {
      tripletMetro.previous_millis = seqMetro.previous_millis;    //sync timers
      synced = true;
    }
  }
  if (beatMaster == 2 && synced) {
    synced = false; //reset sync
  }
  //set a triplet group instantly after the first beat has been played
  //Serial.print("flag: "); Serial.print(tripletFlag);
  //Serial.print("  syncd: "); Serial.print(synced);
  //Serial.print("  beat: "); Serial.print(beatMaster);
  //Serial.print("  tbeat: "); Serial.println(beatMasterTriplet);
  if ( tripletFlag && synced ) {
    setTriplet(obj);
    Serial.println("setting triplet");
  }

  // skip every 4rth beat for triplet groups
  if (triplet) {
    if (beatMasterTriplet == 3 || beatMasterTriplet == 7 || beatMasterTriplet == 11 || beatMasterTriplet == 15) beatMasterTriplet++;
    if (obj.tripletGroup[0] && ( beatHat == 3 || beatHat == 7 || beatHat == 11 || beatHat == 15)) beatHat++;      //triplets use the same array but skip every fourth beat
    if (obj.tripletGroup[1] && ( beatSnare == 3 || beatSnare == 7 || beatSnare == 11 || beatSnare == 15)) beatSnare++;
    if (obj.tripletGroup[2] && ( beatKick == 3 || beatKick == 7 || beatKick == 11 || beatKick == 15)) beatKick++;
    if (obj.tripletGroup[3] && ( beatPerc1 == 3 || beatPerc1 == 7 || beatPerc1 == 11 || beatPerc1 == 15)) beatPerc1++;
    if (obj.tripletGroup[4] && (beatPerc2 == 3 || beatPerc2 == 7 || beatPerc2 == 11 || beatPerc2 == 15)) beatPerc2++;
    if (obj.tripletGroup[5] && (beatSmp1 == 3 || beatSmp1 == 7 || beatSmp1 == 11 || beatSmp1 == 15)) beatSmp1++;
    if (obj.tripletGroup[6] && (beatSmp2 == 3 || beatSmp2 == 7 || beatSmp2 == 11 || beatSmp2 == 15)) beatSmp2++;
    if (obj.tripletGroup[7] && (beatHit == 3 || beatHit == 7 || beatHit == 11 || beatHit == 15)) beatHit++;
  }
  if ( beatMaster >= 16 ) {  //reset master
    beatMaster = 0;
  }
  if ( beatMasterTriplet >= 16 ) {  //reset triplet master
    beatMasterTriplet = 0;
  }
  if ( beatHat >= lastHatBeat )   //reset beats depending on polyrythem
    beatHat = 0;
  if ( beatSnare >= lastSnareBeat )
    beatSnare = 0;
  if ( beatKick >= lastKickBeat )
    beatKick = 0;
  if ( beatPerc1 >= lastPerc1Beat )
    beatPerc1 = 0;
  if ( beatPerc2 >= lastPerc2Beat )
    beatPerc2 = 0;
  if ( beatSmp1 >= lastSmp1Beat )
    beatSmp1 = 0;
  if ( beatSmp2 >= lastSmp2Beat )
    beatSmp2 = 0;
  if ( beatHit >= lastHitBeat ) {
    beatHit = 0;
  }

  if (polyFlag && beatMaster == 1) {   //set polyrythem when master  has rolled over for resyncing
    setPoly();
  }
}

void loadNextBeat(Bar & obj, boolean triplet) {
  if (triplet == obj.tripletGroup[0]) playSamplesBuffer[0] = obj.hatActiveSamples[beatHat];
  if (triplet == obj.tripletGroup[1]) playSamplesBuffer[1] = obj.snareActiveSamples[beatSnare];
  if (triplet == obj.tripletGroup[2]) playSamplesBuffer[2] = obj.kickActiveSamples[beatKick];
  if (triplet == obj.tripletGroup[3]) playSamplesBuffer[3] = obj.perc1ActiveSamples[beatPerc1];
  if (triplet == obj.tripletGroup[4]) playSamplesBuffer[4] = obj.perc2ActiveSamples[beatPerc2];
  if (triplet == obj.tripletGroup[5]) playSamplesBuffer[5] = obj.smp1ActiveSamples[beatSmp1];
  if (triplet == obj.tripletGroup[6]) playSamplesBuffer[6] = obj.smp2ActiveSamples[beatSmp2];
  if (triplet == obj.tripletGroup[7]) playSamplesBuffer[7] = obj.hitActiveSamples[beatHit];
}

void setSwing() {
  if (swingOn) {
    if (beatMaster % 2 == 1)seqMetro.interval(tempoMS - swingMS);
    else seqMetro.interval(tempoMS + swingMS);
  }
} //changed

void randomize(Bar & obj) {
  Serial.print("rando!!!");
  //find the active samples in current selected group and assign them a random sample within their sample scope
  for (int i = 0 ; i < 16 ; i++) {
    if (hatGroupSelect && obj.hatActiveSamples[i] != 0) obj.hatActiveSamples[i] = random(1, 16);
    else if (snareGroupSelect && obj.snareActiveSamples[i] != 0) obj.snareActiveSamples[i] = random(17, 32);
    else if (kickGroupSelect && obj.kickActiveSamples[i] != 0) obj.kickActiveSamples[i] = random(33, 48);
    else if (perc1GroupSelect && obj.perc1ActiveSamples[i] != 0) obj.perc1ActiveSamples[i] = random(49, 64);
    else if (perc2GroupSelect && obj.perc2ActiveSamples[i] != 0) obj.perc2ActiveSamples[i] = random(65, 80);
    else if (sample1GroupSelect && obj.smp1ActiveSamples[i] != 0) obj.smp1ActiveSamples[i] = random(81, 96);
    else if (sample2GroupSelect && obj.smp2ActiveSamples[i] != 0) obj.smp2ActiveSamples[i] = random(97, 112);
    else if (hitGroupSelect && obj.hitActiveSamples[i] != 0) obj.hitActiveSamples[i] = random(113, 128);
  }
}

void randomizeContinuously(Bar & obj) {
  if (hatGroupSelect) obj.shuffleGroup[0] = obj.shuffleGroup[0] ? false : true;
  else if (snareGroupSelect) obj.shuffleGroup[1] = obj.shuffleGroup[1] ? false : true;
  else if (kickGroupSelect) obj.shuffleGroup[2] = obj.shuffleGroup[2] ? false : true;
  else if (perc1GroupSelect) obj.shuffleGroup[3] = obj.shuffleGroup[3] ? false : true;
  else if (perc2GroupSelect) obj.shuffleGroup[4] = obj.shuffleGroup[4] ? false : true;
  else if (sample1GroupSelect) obj.shuffleGroup[5] = obj.shuffleGroup[5] ? false : true;
  else if (sample2GroupSelect) obj.shuffleGroup[6] = obj.shuffleGroup[6] ? false : true;
  else if (hitGroupSelect) obj.shuffleGroup[7] = obj.shuffleGroup[7] ? false : true;
}

void shuffle(Bar & obj) {
  if (obj.shuffleGroup[0])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.hatActiveSamples[i] != 0) obj.hatActiveSamples[i] = random(1, 16);
  if (obj.shuffleGroup[1])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.snareActiveSamples[i] != 0) obj.snareActiveSamples[i] = random(17, 32);
  if (obj.shuffleGroup[2])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.kickActiveSamples[i] != 0) obj.kickActiveSamples[i] = random(33, 48);
  if (obj.shuffleGroup[3])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.perc1ActiveSamples[i] != 0) obj.perc1ActiveSamples[i] = random(49, 64);
  if (obj.shuffleGroup[4])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.perc2ActiveSamples[i] != 0) obj.perc2ActiveSamples[i] = random(65, 80);
  if (obj.shuffleGroup[5])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.smp1ActiveSamples[i] != 0) obj.smp1ActiveSamples[i] = random(81, 96);
  if (obj.shuffleGroup[6])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.smp2ActiveSamples[i] != 0) obj.smp2ActiveSamples[i] = random(97, 112);
  if (obj.shuffleGroup[7])
    for (int i = 0 ; i < 16 ; i++)
      if (obj.hitActiveSamples[i] != 0) obj.hitActiveSamples[i] = random(113, 128);
}

//------------------------------------------------------------------------- group && sample management and other functions ------------------------------------------------------------//

void resetBeat(Bar & obj) {
  beatHat = 0;
  beatSnare = 0;
  beatKick = 0;
  beatPerc1 = 0;
  beatPerc2 = 0;
  beatSmp1 = 0;
  beatSmp2 = 0;
  beatHit = 0;
  beatMaster = 0;
  beatMasterTriplet = 0;

  prevBeatHat = 15;
  prevBeatSnare  = 15;
  prevBeatKick = 15;
  prevBeatPerc1 = 15;
  prevBeatPerc2 = 15;
  prevBeatSmp1  = 15;
  prevBeatSmp2 = 15;
  prevBeatHit = 15;

  lastHatBeat = 16;
  lastSnareBeat = 16;
  lastKickBeat = 16;
  lastPerc1Beat = 16;
  lastPerc2Beat = 16;
  lastSmp1Beat = 16;
  lastSmp2Beat = 16;
  lastHitBeat = 16;

  hatGroupSelect = true;
  snareGroupSelect = false;
  kickGroupSelect = false;
  perc1GroupSelect = false;
  perc2GroupSelect = false;
  sample1GroupSelect = false;
  sample2GroupSelect = false;
  hitGroupSelect = false;

  polyFlag = false;
  tripletFlag = false;
  allTripletFlag = false;
  copyPatternFlag = false;
  fillFlag = false;
  restoreChainFlag = false;
  liveMash = false;
  lockedSamples = true;
  numPatternsSelected = 0;
  copyPtr = 0;
  copyFrom = 0;
  copyTo = 0;
  selectedSampleNumber = 0;
  newSampleSelected = false;
  erasePerformed = false;
  selectedLastBeat = 0;
  lastNormalRollMs = 0;
  lastTripletRollMs = 0;
  swingMS = 0;
  swingOn = false; //changed
  pitchOn = false;
  synced = false;
  fastSynced = false;
  delayGroup[0] = 0;
  delayGroup[1] = 0;
  delayGroup[2] = 0;
  delayGroup[3] = 0;
  delayGroup[4] = 0;
  delayGroup[5] = 0;
  delayGroup[6] = 0;
  delayGroup[7] = 0;
  delayFlag = 0;
  delayTripletFlag = 0;
  lastObj = 0;
  playSamplesTripletDelayBuffer[0] = 0;
  playSamplesTripletDelayBuffer[1] = 0;
  playSamplesTripletDelayBuffer[2] = 0;
  playSamplesTripletDelayBuffer[3] = 0;
  playSamplesTripletDelayBuffer[4] = 0;
  playSamplesTripletDelayBuffer[5] = 0;
  playSamplesTripletDelayBuffer[6] = 0;
  playSamplesTripletDelayBuffer[7] = 0;
  beatBtnPshdRN = 0;
  accentBtnPshdRN = 0;
  polyBtnPshdRN = 0;
  rollBtnPshdRN = 0;
  groupSelectPushedRN = 0;
  tapBtnPushedRN = 0;
  sampleSelectPushedRN = 0;
  erasePushedRN = 0;
  accentPushedRN = 0;
  functionBtnPushedRN = 0;
  trackVolBtnPushedRN = 0;
  polyBtnPushedRN = 0;
  accent = 0;
  poly = 0;
  currentGroup = 0;
  currentSlotPtr = 0;
  tempStorage = 0;
  playSamplesBuffer[0] = 0;
  playSamplesBuffer[1] = 0;
  playSamplesBuffer[2] = 0;
  playSamplesBuffer[3] = 0;
  playSamplesBuffer[4] = 0;
  playSamplesBuffer[5] = 0;
  playSamplesBuffer[6] = 0;
  playSamplesBuffer[7] = 0;
  selectionMem[0] = 0;
  selectionMem[1] = 0;
  selectionMem[2] = 0;
  selectionMem[3] = 0;
  selectionMem[4] = 0;
  selectionMem[5] = 0;
  selectionMem[6] = 0;
  selectionMem[7] = 0;
  lastAccented[0] = 0;
  lastAccented[1] = 0;
  lastAccented[2] = 0;
  lastAccented[3] = 0;
  lastAccented[4] = 0;
  lastAccented[5] = 0;
  lastAccented[6] = 0;
  lastAccented[7] = 0;
  lastSmpl_Accntd[0] = 0;
  lastSmpl_Accntd[1] = 0;
  lastSmpl_Accntd[2] = 0;
  lastSmpl_Accntd[3] = 0;
  lastSmpl_Accntd[4] = 0;
  lastSmpl_Accntd[5] = 0;
  lastSmpl_Accntd[6] = 0;
  lastSmpl_Accntd[7] = 0;
  groupSwitchState[0] = 0;
  groupSwitchState[1] = 0;
  groupSwitchState[2] = 0;
  groupSwitchState[3] = 0;
  groupSwitchState[4] = 0;
  groupSwitchState[5] = 0;
  groupSwitchState[6] = 0;
  groupSwitchState[7] = 0;

  loadNextBeat(obj, false);
  loadNextBeat(obj, true);
  seqMetro.reset();
  tripletMetro.reset();
  wTrig.stopAllTracks();
  wTrig.samplerateOffset(0);
  updateGroupData();
  readVolume(1);  // read vol and tempo values
  readTempo(1);
  //formatTrackVolumes();
  tripletMetro.previous_millis = seqMetro.previous_millis;
}

void changeActiveSamples(int last, int current, Bar & obj) {
  if ( hatGroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.hatActiveSamples[i] != 0)
        obj.hatActiveSamples[i] = hat[current];
  }
  else if ( snareGroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.snareActiveSamples[i] != 0)
        obj.snareActiveSamples[i] = snare[current];
  }
  else if ( kickGroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.kickActiveSamples[i] != 0)
        obj.kickActiveSamples[i] = kick[current];
  }
  else if ( perc1GroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.perc1ActiveSamples[i] != 0)
        obj.perc1ActiveSamples[i] = perc1[current];
  }
  else if ( perc2GroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.perc2ActiveSamples[i] != 0)
        obj.perc2ActiveSamples[i] = perc2[current];
  }
  else if ( sample1GroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.smp1ActiveSamples[i] != 0)
        obj.smp1ActiveSamples[i] = sample1[current];
  }
  else if ( sample2GroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.smp2ActiveSamples[i] != 0)
        obj.smp2ActiveSamples[i] = sample2[current];
  }
  else if ( hitGroupSelect ) {
    for (int i = 0 ; i < 16  ; i++)
      if (obj.hitActiveSamples[i] != 0)
        obj.hitActiveSamples[i] = hit[current];
  }
}

void switchGroup(Bar & obj) {
  Serial.println("Switching group: ");
  copyFromActiveGroup(obj);

  hatGroupSelect = false;
  snareGroupSelect = false;
  kickGroupSelect = false;
  perc1GroupSelect = false;
  perc2GroupSelect = false;
  sample1GroupSelect = false;
  sample2GroupSelect = false;
  hitGroupSelect = false;

  if (currentGroup == 0) {
    hatGroupSelect = true;
    Serial.println("Hats Activated");
  }
  else if (currentGroup == 1) {
    snareGroupSelect = true;
    Serial.println("snares Activated");
  }
  else if (currentGroup == 2) {
    kickGroupSelect = true;
    Serial.println("kicks Activated");
  }
  else if (currentGroup == 3) {
    perc1GroupSelect = true;
    Serial.println("Percs 1 Activated");
  }
  else if (currentGroup == 4) {
    perc2GroupSelect = true;
    Serial.println("Percs 2 Activated");
  }
  else if (currentGroup == 5) {
    sample1GroupSelect = true;
    Serial.println("Smp1 Activated");
  }
  else if (currentGroup == 6) {
    sample2GroupSelect = true;
    Serial.println("Smp2 Activated");
  }
  else if (currentGroup == 7) {
    hitGroupSelect = true;
    Serial.println("Hits Activated");
  }


  //add other groups
  updateGroupData();
  copyToActiveGroup(obj);
}

int getLiveSample(int i) {
  if ( hatGroupSelect )
    return hat[ i ];
  else if ( snareGroupSelect )
    return snare[ i ];
  else if ( kickGroupSelect )
    return kick[ i ];
  else if ( perc1GroupSelect )
    return perc1[ i ];
  else if ( perc2GroupSelect )
    return perc2[ i ];
  else if ( sample1GroupSelect )
    return sample1[ i ];
  else if ( sample2GroupSelect )
    return sample2[ i ];
  else if ( hitGroupSelect )
    return hit[ i ];
}

void setPoly() {
  Serial.println("setPoly");
  if ( hatGroupSelect ) {
    lastHatBeat = selectedLastBeat + 1;
    beatHat = 1;
  }
  else if ( snareGroupSelect ) {
    lastSnareBeat = selectedLastBeat + 1;
    beatSnare = 1;
  }
  else if ( kickGroupSelect ) {
    lastKickBeat = selectedLastBeat + 1;
    beatKick = 1;
  }
  else if ( perc1GroupSelect ) {
    lastPerc1Beat = selectedLastBeat + 1;
    beatPerc1 = 1;
  }
  else if ( perc2GroupSelect ) {
    lastPerc2Beat = selectedLastBeat + 1;
    beatPerc2 = 1;
  }
  else if ( sample1GroupSelect ) {
    lastSmp1Beat = selectedLastBeat + 1;
    beatSmp1 = 1;
  }
  else if ( sample2GroupSelect ) {
    lastSmp2Beat = selectedLastBeat + 1;
    beatSmp2 = 1;
  }
  else if ( hitGroupSelect ) {
    lastHitBeat = selectedLastBeat + 1;
    beatHit = 1;
  }
  polyFlag = false;
}

int selectedBeat() {
  if (hatGroupSelect) return prevBeatHat;
  else if (snareGroupSelect) return prevBeatSnare;
  else if (kickGroupSelect) return prevBeatKick;
  else if (perc1GroupSelect) return prevBeatPerc1;
  else if (perc2GroupSelect) return prevBeatPerc2;
  else if (sample1GroupSelect) return prevBeatSmp1;
  else if (sample2GroupSelect) return prevBeatSmp2;
  else if (hitGroupSelect) return prevBeatHit;
}

void copyToActiveGroup(Bar & obj) {     // after the group has been switched, this function is run to copy the specific group arrays back the the activeGroup
  if (hatGroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsHat & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.hatAccents & (1 << i) ? 1 : 0;
    }
  }
  if (snareGroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsSnare & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.snareAccents & (1 << i) ? 1 : 0;
    }
  }
  if (kickGroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsKick & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.kickAccents & (1 << i) ? 1 : 0;
    }
  }
  if (perc1GroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsPerc1 & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.perc1Accents & (1 << i) ? 1 : 0;
    }
  }
  if (perc2GroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsPerc2 & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.perc2Accents & (1 << i) ? 1 : 0;
    }
  }
  if (sample1GroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsSmp1 & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.smpAccents & (1 << i) ? 1 : 0;
    }
  }
  if (sample2GroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsSmp2 & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.smp2Accents & (1 << i) ? 1 : 0;
    }
  }
  if (hitGroupSelect) {
    for (int i = 0; i < 16; i++) {
      buttonsActive[i] = obj.buttonsHit & (1 << i) ? 1 : 0;
      accentBtnsActive[i] = obj.hitAccents & (1 << i) ? 1 : 0;
    }
  }
}

void copyFromActiveGroup(Bar & obj) {       // initiated before the group has been switched - copies buttonsActive array to buttonsHat (or other specific group) array memory
  if (hatGroupSelect) {
    obj.buttonsHat = 0;
    obj.hatAccents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsHat |= 1 << i;
      if (accentBtnsActive[i])
        obj.hatAccents |= 1 << i;
    }
  }
  if (snareGroupSelect) {
    obj.buttonsSnare = 0;
    obj.snareAccents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsSnare |= 1 << i;
      if (accentBtnsActive[i])
        obj.snareAccents |= 1 << i;
    }
  }
  if (kickGroupSelect) {
    obj.buttonsKick = 0;
    obj.kickAccents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsKick |= 1 << i;
      if (accentBtnsActive[i])
        obj.kickAccents |= 1 << i;
    }
  }
  if (perc1GroupSelect) {
    obj.buttonsPerc1 = 0;
    obj.perc1Accents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsPerc1 |= 1 << i;
      if (accentBtnsActive[i])
        obj.perc1Accents |= 1 << i;
    }
  }
  if (perc2GroupSelect) {
    obj.buttonsPerc2 = 0;
    obj.perc2Accents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsPerc2 |= 1 << i;
      if (accentBtnsActive[i])
        obj.perc2Accents |= 1 << i;
    }
  }
  if (sample1GroupSelect) {
    obj.buttonsSmp1 = 0;
    obj.smpAccents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsSmp1 |= 1 << i;
      if (accentBtnsActive[i])
        obj.smpAccents |= 1 << i;
    }
  }
  if (sample2GroupSelect) {
    obj.buttonsSmp2 = 0;
    obj.smp2Accents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsSmp2 |= 1 << i;
      if (accentBtnsActive[i])
        obj.smp2Accents |= 1 << i;
    }
  }
  if (hitGroupSelect) {
    obj.buttonsHit = 0;
    obj.hitAccents = 0;
    for (int i = 0; i < 16; ++i) {
      if (buttonsActive[i])
        obj.buttonsHit |= 1 << i;
      if (accentBtnsActive[i])
        obj.hitAccents |= 1 << i;
    }
  }
}
void addRollSample(int beat, Bar & obj) {
  if ( hatGroupSelect ) obj.rollSamples[beat] = 1;
  if ( snareGroupSelect ) obj.rollSamples[beat] = 2;
  if ( kickGroupSelect ) obj.rollSamples[beat] = 3;
  if ( perc1GroupSelect ) obj.rollSamples[beat] = 4;
  if ( perc2GroupSelect ) obj.rollSamples[beat] = 5;
  if ( sample1GroupSelect ) obj.rollSamples[beat] = 6;
  if ( sample2GroupSelect ) obj.rollSamples[beat] = 7;
  if ( hitGroupSelect ) obj.rollSamples[beat] = 8;
  for ( int i = 0 ; i < 16 ; i++)
    Serial.print( obj.rollSamples[i]);
  Serial.println();
}
void removeRollSample(int beat, Bar & obj) {
  //beat = beat == 15 ? 0 : beat + 1;
  obj.rollSamples[beat] = 0;
  for ( int i = 0 ; i < 16 ; i++)
    Serial.print( obj.rollSamples[i]);
  Serial.println();
}
void addSample(int beat, Bar & obj) {
  if (hatGroupSelect) {
    obj.hatActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (snareGroupSelect) {
    obj.snareActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (kickGroupSelect) {
    obj.kickActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (perc1GroupSelect) {
    obj.perc1ActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (perc2GroupSelect) {
    obj.perc2ActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (sample1GroupSelect) {
    obj.smp1ActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (sample2GroupSelect) {
    obj.smp2ActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
  if (hitGroupSelect) {
    obj.hitActiveSamples[beat - 1] = getCrrntSlctdSmpl();
  }
}

void removeSample(int beat, Bar & obj) {
  if (hatGroupSelect) {
    obj.hatActiveSamples[beat - 1] = 0;
  }
  if (snareGroupSelect) {
    obj.snareActiveSamples[beat - 1] = 0;
  }
  if (kickGroupSelect) {
    obj.kickActiveSamples[beat - 1] = 0;
  }
  if (perc1GroupSelect) {
    obj.perc1ActiveSamples[beat - 1] = 0;
  }
  if (perc2GroupSelect) {
    obj.perc2ActiveSamples[beat - 1] = 0;
  }
  if (sample1GroupSelect) {
    obj.smp1ActiveSamples[beat - 1] = 0;
  }
  if (sample2GroupSelect) {
    obj.smp2ActiveSamples[beat - 1] = 0;
  }
  if (hitGroupSelect) {
    obj.hitActiveSamples[beat - 1] = 0;
  }
}

int getCrrntSlctdSmpl() {
  if ( hatGroupSelect )
    return hat[ selectionMem[currentGroup] ];
  else if ( snareGroupSelect )
    return snare[ selectionMem[currentGroup] ];
  else if ( kickGroupSelect )
    return kick[ selectionMem[currentGroup] ];
  else if ( perc1GroupSelect )
    return perc1[ selectionMem[currentGroup] ];
  else if ( perc2GroupSelect )
    return perc2[ selectionMem[currentGroup] ];
  else if ( sample1GroupSelect )
    return sample1[ selectionMem[currentGroup] ];
  else if ( sample2GroupSelect )
    return sample2[ selectionMem[currentGroup] ];
  else
    return hit[ selectionMem[currentGroup] ];
}

void formatTrackVolumes() {
  for ( int i = 0 ; i < NUM_TRACKS ; i++)
    wTrig.trackGain(i, 0);
}
void setTrackVolumes() {
  for ( int i = 0 ; i < NUM_TRACKS ; i++)
    wTrig.trackGain(i, trackVolumes[i]);
}

void setTriplet(Bar & obj) {
  Serial.println("                Setting Triplet");
  if (allTripletFlag) {
    if (obj.tripletGroup[0] == 1 && obj.tripletGroup[1] == 1 && obj.tripletGroup[2] == 1 && obj.tripletGroup[3] == 1 && obj.tripletGroup[4] == 1 && obj.tripletGroup[5] == 1)  //if all groups already high turn all off
      for (int i = 0; i < 8; i++)
        obj.tripletGroup[i] = 0;
    else                                                              // otherwise turn all groups on
      for (int i = 0; i < 8; i++)
        obj.tripletGroup[i] = 1;
  }
  else if ( hatGroupSelect )                                // turn on triplet groups individually
    obj.tripletGroup[0] = obj.tripletGroup[0] ? false : true;
  else if ( snareGroupSelect )
    obj.tripletGroup[1] = obj.tripletGroup[1] ? false : true;
  else if ( kickGroupSelect )
    obj.tripletGroup[2] = obj.tripletGroup[2] ? false : true;
  else if ( perc1GroupSelect )
    obj.tripletGroup[3] = obj.tripletGroup[3] ? false : true;
  else if ( perc2GroupSelect )
    obj.tripletGroup[4] = obj.tripletGroup[4] ? false : true;
  else if ( sample1GroupSelect )
    obj.tripletGroup[5] = obj.tripletGroup[5] ? false : true;
  else if ( sample2GroupSelect )
    obj.tripletGroup[6] = obj.tripletGroup[6] ? false : true;
  else if ( hitGroupSelect )
    obj.tripletGroup[7] = obj.tripletGroup[7] ? false : true;
  tripletFlag = false; allTripletFlag = false;
}

//==============================================================================================MEMORY=====================================================================================================//
void save(Bar & obj) {    // (4KB total mem in ROM)
  int selection = getSelection(obj, true);   // get a input from the user
  if(selection == 100){
    updateLCD(true, tempoBPM, true);      // update lcd
    return;      // if tap button was pressed we revieve 100, our terminate sequence number
  }
  lcd.home();
  lcd.print("   saving to    ");
  lcd.setCursor(0, 1);
  lcd.print("    slot ");
  lcd.print(selection);
  lcd.print("     ");
  copyFromActiveGroup(obj);  // copy current selected beats to storage arrays(hatActiveSamples etc) for beat led display/button action
  memcpy(&memory.a_ROM, &a, sizeof(a));    // ~1136 total bytes per program (4KB total internal EEPROM 32KB external EEPROM)
  memcpy(&memory.b_ROM, &b, sizeof(b));
  memcpy(&memory.c_ROM, &c, sizeof(c));
  memcpy(&memory.f_ROM, &f, sizeof(f));
  memcpy(memory.trackVolumes_ROM, trackVolumes, sizeof(trackVolumes));    // ~288 total bytes per program (4KB total)
  memcpy(memory.chain_ROM, chain, sizeof(chain));

  int totalSizeInBytes = 4 * sizeof(a) + sizeof(trackVolumes) + sizeof(chain);
  int totalBytes = EEPROM_writeAnything(totalSizeInBytes * selection, memory);   //save the config structure into eeprom

  memcpy(&savedPatternNames[selection][0], &saveName[0], 16);     // save the user made name into the array of stored names
  EEPROM_writeAnything(totalSizeInBytes * 16, savedPatternNames); // save the pattern names

  Serial.print("Saved: "); Serial.print(totalBytes); Serial.print(" bytes to slot: "); Serial.println(selection);
  animate(selection, 1);    // trigger save/load animation ( 0 for load , 1 for save - trigger different animations )
  resetBeat(obj);    // start the beat over
  updateLCD(true, tempoBPM, true);
}
void load(Bar & obj) {
  Serial.println("load initiated");
  int totalBytes;
  int selection = getSelection(obj, false);   // get a input from the user
  if(selection == 100){
    updateLCD(true, tempoBPM, true);  // update lcd
    return;      // if tap button was pressed we revieve 100, our terminate sequence number
  }
  lcd.home();
  lcd.print("  loading from  ");
  lcd.setCursor(0, 1);
  lcd.print("    slot ");
  lcd.print(selection);
  lcd.print("     ");
  int totalSizeInBytes = 4 * sizeof(a) + sizeof(trackVolumes);
  totalBytes = EEPROM_readAnything(900 * selection, memory); // load data from eeprom into structure and save total data size for printing
  memcpy(&a, &memory.a_ROM, sizeof(a));    // ~900 total bytes per program (4KB total internal EEPROM 32KB external EEPROM)
  memcpy(&b, &memory.b_ROM, sizeof(b));
  memcpy(&c, &memory.c_ROM, sizeof(c));
  memcpy(&f, &memory.f_ROM, sizeof(f));
  memcpy(trackVolumes, memory.trackVolumes_ROM, sizeof(trackVolumes)); // Bytes = numbtracks * 2
  memcpy(chain, memory.chain_ROM, sizeof(chain));

  copyToActiveGroup(obj); // copy the selected stored group to the active group
  Serial.print("Loaded: "); Serial.print(totalBytes); Serial.print(" bytes from slot: "); Serial.println(selection);
  animate(selection, 0);   // trigger save/load animation ( 0 for load , 1 for save - trigger different animations )
  setTrackVolumes();
  resetBeat(obj);    // start the beat over
  updateLCD(true, tempoBPM, true);
}

int getSelection(Bar & obj, boolean saving) {
  lcd.setCursor(0, 0);    // display save/load
  if (saving)lcd.print("save initiated  ");
  else lcd.print("load initiated  ");
  lcd.setCursor(0, 1);    // clear lcd bottom row
  lcd.print("                ");
  boolean selectionMade = 0;  // declare local variables
  boolean beatBtnPushed = true;
  int selection;
  int slotSelKnob = 0;
  int slotSelKnobAnalog = 0;
  int slotSelKnobAnalogPrev = 0;
  int lastSlotSelKnob = 0;
  byte slotSelBinary = 0;
  long currentTime = millis();
  byte cycle = 0;
  int analogLetter = 0;
  int lastAnalogLetter = 1;
  byte currentLetterPlace = 0;
  boolean sampleBtnState = false;
  boolean done = false;
  byte selectionBinary;
  int displayTime = 3000;
  boolean firstScan = true;
  while (functionBtnPushedRN) {       // wait until the function button has been released before continuing
    readBtn_Function(obj); delay(10);
  }
  while (beatBtnPushed) {         // wait until the beat button has been released before continuing
    beatBtnPushed = false;
    for (int i = 0; i < 16 ; i++)
      if ( digitalRead(buttonPin[i]) == LOW)
        beatBtnPushed = true;
    delay(10);
  }
  while (!selectionMade) {    // rotate knob for selection and read pattern names to lcd
    if( !digitalRead(tapBtnPin)) return 100;      // use "tap" button to esc sequence
    slotSelKnobAnalog = analogRead(groupPin);
    if (abs(slotSelKnobAnalog - slotSelKnobAnalogPrev) > 5) {   // only convert the 0-1023 val to 0-16 if has changed signigicantly
      slotSelKnob = map(slotSelKnobAnalog, 0, 1015, 0, 15); //read group knob and translate to 0-15 scale for cooresponding leds
      slotSelKnobAnalogPrev = slotSelKnobAnalog;
    }
    if ( slotSelKnob != lastSlotSelKnob || firstScan) {      // when current slot has changed we update the led and pattern name on lcd
      lcd.setCursor(0, 1);        // clear lcd of prev name
      lcd.print("                ");
      lcd.setCursor(0, 1);
      for (int i = 0 ; i < 16 ; i++) {      // print the track name character by character
        lcd.print(savedPatternNames[slotSelKnob][i]);
      }
      if (slotSelKnob < 8) {      // display selection on leds
        slotSelBinary = 1 << slotSelKnob;
        beatLeds( slotSelBinary , 0);
      } else {
        slotSelBinary = 1 << slotSelKnob - 8;
        beatLeds(0,  slotSelBinary);
      }
      lastSlotSelKnob = slotSelKnob;
    }
    if ( digitalRead(sampleTogglePin) == LOW) { // confirm selection with sample button
      selection = slotSelKnob;
      selectionMade = true;
      Serial.print("Selection1:"); Serial.println(selection);
      break;
    }
    if (cycle == 1) displayTime = 4000;   // hold longer when displaying "save/load initiated"
    else displayTime = 1500;
    if (millis() - currentTime > displayTime) {      // cycle top row of lcd with instructions
      currentTime = millis();
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      if ( cycle == 0) {
        if (saving)lcd.print("save initiated  ");
        else lcd.print("load initiated");
      }
      else if ( cycle == 1)lcd.print("browse slots");
      else if ( cycle == 2)lcd.print("with group knob");
      else if ( cycle == 3)lcd.print("confirm sel with");
      else if ( cycle == 4)lcd.print("sample button");
      else if ( cycle == 5)lcd.print("tap btn to esc");
      cycle++;
      cycle = cycle > 5 ? 0 : cycle;
    }
    firstScan = false;
  }
  while ( digitalRead(sampleTogglePin) == LOW ) { // wait until the sample button has been released before continuing
    delay(10);
  } delay(100);     // debounce
  if (!saving) return selection;      // if loading then we are done
  lcd.setCursor(0, 0);      // display instructions in lcd
  lcd.print("enter track name");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  if (selection < 8) {
    selectionBinary = 1 << selection;
    beatLeds( slotSelBinary , 0);
  } else {
    selectionBinary = 1 << selection - 8;
    beatLeds(0,  selectionBinary);
  }
  currentTime = millis();     // reset lcd cycle variables
  cycle = 1;
  displayTime = 4000;
  while (!done) {            // name the track!
    if( !digitalRead(tapBtnPin)) return 100;      // use "tap" button to esc sequence
    analogLetter = analogRead(groupPin);
    analogLetter = map(analogLetter, 0, 1015, 0, 26) + 96; //read group knob and translate to 0-26 scale for letters
    if ( analogLetter != lastAnalogLetter) {  //if scrolled to a new letter print it and put in array
      lcd.setCursor(currentLetterPlace, 1);
      if(analogLetter == 96) analogLetter = 32;   // we use space char(32 in ascii) for knob left
      lcd.print((char)analogLetter);
      saveName[currentLetterPlace] = analogLetter;
      lastAnalogLetter = analogLetter;
    }
    if ( !digitalRead(sampleTogglePin)  && !sampleBtnState) { // if sample button is pushed move the cursor over
      sampleBtnState = true;
      currentLetterPlace++;
      currentLetterPlace = currentLetterPlace == 16 ? 0 : currentLetterPlace;
      lastAnalogLetter++;       // triggers an update for new letter when a letter is entered
      delay(100);
    }
    if ( digitalRead(sampleTogglePin) ) {   // debounce the sample button and reset state when released
      sampleBtnState = false;
      delay(100);
    }
    if (!digitalRead(buttonPin[selection])) { // if corresponding button is pushed again we are done
      done = true;
    }
    if (cycle == 1) displayTime = 4000;   // hold longer when displaying "save/load initiated"
    else displayTime = 1500;
    if (millis() - currentTime > displayTime) {      // cycle top row of lcd with instructions
      currentTime = millis();
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      if ( cycle == 0) lcd.print("enter track name");
      else if ( cycle == 1)lcd.print("use group knob");
      else if ( cycle == 2)lcd.print("and sample btn");
      else if ( cycle == 3)lcd.print("finalize with");
      else if ( cycle == 4)lcd.print("lit up beat btn");
      else if ( cycle == 5)lcd.print("tap btn to esc");
      cycle++;
      cycle = cycle > 5 ? 0 : cycle;
    }
  }
  if (currentLetterPlace != 15) {     // if not 16 letters, fill in the rest of the character elements with spaces
    for (int i = currentLetterPlace ; i < 16 ; i++) {
      saveName[i] = ' ';
    }
  }
  return selection;
}
void writeEEPROM(unsigned int eeaddress, byte data )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}
byte readEEPROM( unsigned int eeaddress )
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

//-iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii-IO SCAN-ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo-/
//---------------------- Analog -------------------//
void checkAnalogIO(Bar & obj) {
  if (readAnalogInputs.check() == 1) {  // if minumum time has passed the inputs are read
    readTempo(0);
    readVolume(0);
    readGroup(0, obj);
    if ( trackVolBtnPushedRN == 1 && !accent) readTrackVolume(0, obj);
    if (pitchOn == 1) readSampleRate(0);
    if (swingOn) readSwing(0);
    if(accent) readAccentVolume();
  }
}

void readSampleRate(boolean firstCall) {
  static int sampleIntermediate;
  static int sampleLastIntermediate;
  static int sampleRate;
  //Serial.println("Checking SampleRate");
  sampleIntermediate = analogRead(sampleRatePin);
  if ( abs( sampleLastIntermediate - sampleIntermediate) > 10 || firstCall) {
    sampleLastIntermediate = sampleIntermediate;
    sampleRate = map(sampleIntermediate, 0, 1023, -32767, 32767);
    wTrig.samplerateOffset(sampleRate);
  }
}
void readSwing(boolean firstCall) {
  static int swingIntermediate;
  static int swingLastIntermediate;
  static int swingAmnt;
  //Serial.println("Checking swingAmnt");
  swingIntermediate = analogRead(swingPin);
  if ( abs( swingLastIntermediate - swingIntermediate) > 10 || firstCall) {
    swingLastIntermediate = swingIntermediate;
    swingAmnt = map(swingIntermediate, 0, 1023, 0, 51);  //scale to 0-50
    if (swingAmnt > 50) swingAmnt = 50;
    swingMS = swingAmnt * tempoMS / 100; // get 0% to 50% of MS between beats
    int percent = (tempoMS + swingMS) * 100 / (tempoMS * 2); // calculate percent based on Linns method - 50% is no swing, equal distribution between odd and even beats - top range is 75%
    //Serial.print("TempoMs: ");Serial.print(tempoMS);Serial.print("  Swing Rate: "); Serial.print(swingAmnt);Serial.print("   swingMS: ");Serial.print(swingMS);Serial.print("   Percent: ");Serial.println(percent);
  }
}
void readTrackVolume(boolean firstCall, Bar & obj) {      // all track volumes are initially set to 0
  // read analog input for volume and map to wav trigger vol range (-70 to +10)
  static int trackVolumeLastIntermediate;
  static int trackVolumeIntermediate;
  static int trackVolume;
  trackVolumeIntermediate = analogRead(trckVolPn);
  if ( abs( trackVolumeLastIntermediate - trackVolumeIntermediate) > 10 || firstCall) {
    trackVolumeLastIntermediate = trackVolumeIntermediate;
    trackVolume = map(trackVolumeIntermediate, 0, 1023, -30, 10);   // tracks can go to -70 but are pretty much inaudible by -30
    wTrig.trackGain(getCrrntSlctdSmpl(), trackVolume);
    trackVolumes[getCrrntSlctdSmpl()] = trackVolume;
  }
}

void readAccentVolume(){      
  // when accent button is pushed, the track vol knob is used to set the accent vs non accent vol difference
  static int accentVolIntermediate;
  accentVolIntermediate = analogRead(trckVolPn);
  if ( abs( accentVolLastIntermediate - accentVolIntermediate) > 10) {
    accentVolLastIntermediate = accentVolIntermediate;
    accentDBOffset = map(accentVolIntermediate, 0, 1023, 0, 30); 
    Serial.print("Accent difference set to: ");
    Serial.println(accentDBOffset);
  }
}

void readVolume(boolean firstCall) {      // read analog input for volume and map to wav trigger vol range (-70 to +4)
  static int volumeLastIntermediate;
  static int volumeIntermediate;
  volumeIntermediate = analogRead(volumePin);
  if ( abs( volumeLastIntermediate - volumeIntermediate) > 10 || firstCall) {
    volumeLastIntermediate = volumeIntermediate;
    volume = map(volumeIntermediate, 0, 1023, -70, 4);  // -70 to 4 DB is the full range of the wav trigger
    wTrig.masterGain(volume);
    Serial.print("Volume: "); Serial.println(volume);
  }
}
void readTempo(boolean firstCall) {
  // read analog input for tempo - if significantly changed update the tempo
  static int tempoIntermediate;
  static int tempoLastIntermediate;
  tempoIntermediate = analogRead(tempoPin);
  if ( abs( tempoLastIntermediate - tempoIntermediate ) > 10 || firstCall) {
    tempoLastIntermediate = tempoIntermediate;
    tempoMS = map(tempoIntermediate, 1023, 0, 83, 320);
    seqMetro.interval(tempoMS);
    halfMS = tempoMS / 2;
    tripletMS = tempoMS * 4 / 3;
    halfTripletMS = tripletMS / 2;
    tripletMetro.interval(tripletMS);
    tempoBPM = 60000 / (tempoMS * 4);   /// ms to BPM conversion, the 4 is there because every DB "beat" is actually an eight note
    updateLCD(false, tempoBPM, false); // display new tempo on lcd, update only the BPM
  }
}
void readGroup(boolean firstCall, Bar & obj) {
  // read analog input for group numbers 1 to 8
  static int groupLastIntermediate;
  static int groupIntermediate;
  groupIntermediate = analogRead(groupPin);
  if ( abs( groupLastIntermediate - groupIntermediate) > 10 || firstCall) {
    groupLastIntermediate = groupIntermediate;
    currentGroup = map(groupIntermediate, 0, 1023, 0, 8);  // -70 to 4 DB is the full range of the wav trigger
    if ( currentGroup == 8 ) currentGroup = 7;
    if (currentGroup != lastGroup) {
      switchGroup(obj);
      lastGroup = currentGroup;
      updateLCD(true, 0, false); // update the sample name only
    }
  }
}
//---------------------- Digital -------------------//
void checkDigitalIO(Bar & obj) {
  if (readDigitalInputsHP.check() == 1) {
    if (accent && poly) {
      writeRollLEDs();
      checkRollButtons(obj);
    } else if (accent) {
      writeAccentLEDs();
      checkAccentButtons(obj);
    } else if (poly) {
      writePolyLEDs();
      checkPolyButtons();
    } else if (functionBtnPushedRN) {
      writePatternLEDs();
      checkBeatButtons(obj);
    } else {
      checkBeatButtons(obj);
      writeBeatLEDs();
    }
    checkHighPriorityButtons(obj);
    checkMedPriorityButtons(obj);
  }
  if (readDigitalInputsLP.check() == 1) {
    checkLowPriorityButtons(obj);
    readSampleRateSwitch();
    readSwingSwitch();
    checkGroupSwitches();
    readStutterSwitch();
    readPauseSwitch();
    //Serial.print(numPatternsSelected); //NOTE
  }
}

void checkGroupSwitches() {
  for ( int j = 0 ; j < 8 ; j++) {
    if (digitalRead(gSwitchPin[j]) == HIGH && groupSwitchState[j]) {
      groupSwitchState[j] = 0;
    }
    if (digitalRead(gSwitchPin[j]) == LOW && !groupSwitchState[j]) {
      groupSwitchState[j] = 1;
    }
  }
}

void readSampleRateSwitch() {
  if ( digitalRead(pitchSwitchPin) == LOW && !pitchOn) {
    Serial.println("Pitch ON");//
    readSampleRate(1);    //read sample rate and update no matter what
    pitchOn = 1;
  }
  if (digitalRead(pitchSwitchPin) == HIGH && pitchOn) {
    Serial.println("Pitch OFF");
    wTrig.samplerateOffset(0);
    pitchOn = 0;
  }
}
void readSwingSwitch() {
  if ( digitalRead(swingSwitchPin) == LOW && !swingOn) {
    Serial.println("Swing ON");//
    readSwing(1);
    swingOn = 1;
  }
  if (digitalRead(swingSwitchPin) == HIGH && swingOn) {
    Serial.println("Swing OFF");
    seqMetro.interval(tempoMS);
    swingOn = 0;
    swingMS = 0; //reset swing rate to 0 while switch is off
  }
}
void readStutterSwitch() {
  if ( digitalRead(stutterSwitchPin) == LOW && !stutterOn) {
    stutterOn = true;
    Serial.println("stutter ON");
  }
  if (digitalRead(stutterSwitchPin) == HIGH && stutterOn && beatMaster == 15) {
    stutterOn = false;
    Serial.println("stutter OFF");
  }
}
void readPauseSwitch() {
  if ( digitalRead(pauseSwitchPin) == LOW && !pauseOn) {
    pauseOn = true;
    Serial.println("pause ON");
  }
  if (digitalRead(pauseSwitchPin) == HIGH && pauseOn) {
    pauseOn = false;
    Serial.println("pause OFF");
  }
}
void checkLowPriorityButtons(Bar & obj) {     // check buttons by priority
  readBtn_Function(obj);
}
void checkMedPriorityButtons(Bar & obj) {
  readBtn_Sample(obj);
  readBtn_Erase(obj);
  readBtn_Accent(obj);
  readBtn_TrackVol(obj);
  readBtn_Poly(obj);
}
void checkHighPriorityButtons(Bar & obj) {
  readBtn_Tap(obj);
}

void readBtn_Function(Bar & obj) {
  if ( digitalRead(functionBtnPin) == LOW && !functionBtnPushedRN ) {
    functionBtnPushedRN = 1;
    Serial.println("PATTERN PUSHED");
    //save(obj);

  } else if (digitalRead(functionBtnPin) == HIGH && functionBtnPushedRN) {
    functionBtnPushedRN = 0;
    if (numPatternsSelected == 1) {
      // reset elements that wont be included in the chain
      chain[1] = 0; chain[2] = 0; chain[3] = 0;
      //point slot to first element in chain every time a new pat is selected
      currentSlotPtr = 0;
    }
    else if (numPatternsSelected == 2) {
      // reset elements that wont be included in the chain
      chain[2] = 0; chain[3] = 0;
      //point slot to first element in chain every time a new pat is selected
      currentSlotPtr = 0;
    }
    else if (numPatternsSelected == 3) {
      // reset elements that wont be included in the chain
      chain[3] = 0;
      //point slot to first element in chain every time a new pat is selected
      currentSlotPtr = 0;
    }
    numPatternsSelected = 0;
    copyPtr = 0;
    if (copyPatternFlag)copyPatternFlag = false;  //reset copy Pattern flag
    Serial.println("PATTERN RELEASED");
  }
}
void readBtn_Tap(Bar & obj) {
  if ( digitalRead(tapBtnPin) == LOW && !tapBtnPushedRN) {    // add samples with tap button
    Serial.print("TAP BTN");
    tapBtnPushedRN = 1;
    reading = millis();
    if (!tripletActiveforCrntGrp(obj)) {              // if triplets not active for current beat
      if (reading - startTime < halfMS) {           // add tap button to the closest beat
        buttonsActive[prevBeatHat] = 1;
        addSample(prevBeatHat + 1, obj);             //adds sample to corresponding beat
      }
      else {
        buttonsActive[beatHat] = 1;
        addSample(beatHat + 1, obj);
      }
    }
    else {                                    //if triplets active for current beat adjust the add timing
      if (reading - startTimeTriplet < halfTripletMS) {           // add tap button to the closest beat
        buttonsActive[prevBeatHat] = 1;
        addSample(prevBeatHat + 1, obj);             //adds sample to corresponding beat
      }
      else {
        buttonsActive[beatHat] = 1;
        addSample(beatHat + 1, obj);
      }
    }
    writeBeatLEDs();
  }
  else if (digitalRead(tapBtnPin) == HIGH && tapBtnPushedRN) {
    tapBtnPushedRN = 0;
  }
}
boolean tripletActiveforCrntGrp(Bar & obj) {   // are triplets active for the current selected group?
  if ( hatGroupSelect && obj.tripletGroup[0])
    return true;
  else if ( snareGroupSelect && obj.tripletGroup[1])
    return true;
  else if ( kickGroupSelect && obj.tripletGroup[2])
    return true;
  else if ( perc1GroupSelect && obj.tripletGroup[3])
    return true;
  else if ( perc2GroupSelect && obj.tripletGroup[4])
    return true;
  else if ( sample1GroupSelect && obj.tripletGroup[5])
    return true;
  else if ( sample2GroupSelect && obj.tripletGroup[6])
    return true;
  else if ( hitGroupSelect && obj.tripletGroup[7])
    return true;
  else
    return false;
}


void readBtn_Sample(Bar & obj) {
  if ( digitalRead(sampleTogglePin) == LOW && !sampleSelectPushedRN) {    // sample select button
    sampleSelectPushedRN = 1;
    Serial.print("selection MemA:"); Serial.println(selectionMem[currentGroup]);
  }
  else if (digitalRead(sampleTogglePin) == HIGH && sampleSelectPushedRN) {
    sampleSelectPushedRN = 0;
    int lastSample = selectionMem[currentGroup];
    if ( !newSampleSelected ) {
      if (selectionMem[currentGroup] < 9)
        selectionMem[currentGroup]++;
      else
        selectionMem[currentGroup] = 0;
    }
    if (lockedSamples)changeActiveSamples(lastSample, selectionMem[currentGroup], obj);
    //sevenSeg(selectionMem[currentGroup]);
    newSampleSelected = false;
    updateLCD(true, 0, false); // update the sample name only
    Serial.print("selection MemB:"); Serial.println(selectionMem[currentGroup]);
  }
}

void readBtn_Erase(Bar & obj) {
  if ( digitalRead(eraseBtnPin) == LOW && !erasePushedRN) {    // erase button
    erasePushedRN = 1;
    Serial.println("erase button pushed");
  }
  else if (digitalRead(eraseBtnPin) == HIGH && erasePushedRN) {
    erasePushedRN = 0;
    updateLCD(true, tempoBPM, true); // update data after displaying erase instructions
    Serial.println("erase button released");
    for ( int i = 0 ; i < 16 ; i++) {
      buttonsActive[i] = 0;
      if (erasingHats || erasingAll) {
        Serial.print("ErasingHats");
        obj.hatActiveSamples[i] = 0;
        obj.hatAccents = 0;
        obj.buttonsHat = 0;
        if ( obj.rollSamples[i] <= 16)
          obj.rollSamples[i] = 0;
      }
      if (erasingSnares || erasingAll) {
        obj.snareActiveSamples[i] = 0;
        obj.snareAccents = 0;
        obj.buttonsSnare = 0;
        if ( obj.rollSamples[i] > 16 && obj.rollSamples[i] <= 32)
          obj.rollSamples[i] = 0;
      }
      if (erasingKicks || erasingAll) {
        obj.kickActiveSamples[i] = 0;
        obj.kickAccents = 0;
        obj.buttonsKick = 0;
        if ( obj.rollSamples[i] > 32 && obj.rollSamples[i] <= 48)
          obj.rollSamples[i] = 0;
      }
      if (erasingPerc1 || erasingAll) {
        obj.perc1ActiveSamples[i] = 0;
        obj.perc1Accents = 0;
        obj.buttonsPerc1 = 0;
        if ( obj.rollSamples[i] > 48 && obj.rollSamples[i] <= 64)
          obj.rollSamples[i] = 0;
      }
      if (erasingPerc2 || erasingAll) {
        obj.perc2ActiveSamples[i] = 0;
        obj.perc2Accents = 0;
        obj.buttonsPerc2 = 0;
        if ( obj.rollSamples[i] > 64 && obj.rollSamples[i] <= 80)
          obj.rollSamples[i] = 0;
      }
      if (erasingSmp1 || erasingAll) {
        obj.smp1ActiveSamples[i] = 0;
        obj.smpAccents = 0;
        obj.buttonsSmp1 = 0;
        if ( obj.rollSamples[i] > 80 && obj.rollSamples[i] <= 96)
          obj.rollSamples[i] = 0;
      }
      if (erasingSmp2 || erasingAll) {
        obj.smp2ActiveSamples[i] = 0;
        obj.smp2Accents = 0;
        obj.buttonsSmp2 = 0;
        if ( obj.rollSamples[i] > 112 && obj.rollSamples[i] <= 128)
          obj.rollSamples[i] = 0;
      }
      if (erasingHits || erasingAll) {
        obj.hitActiveSamples[i] = 0;
        obj.hitAccents = 0;
        obj.buttonsHit = 0;
        if ( obj.rollSamples[i] > 96 && obj.rollSamples[i] <= 112)
          obj.rollSamples[i] = 0;
      }
      if ( erasingAll ) {
        erasePerformed = true;
        buttonsActive[i] = 0;
        accentBtnsActive[i] = 0;
      }
    }
    writeAccentLEDs();
    erasingHats = false;
    erasingSnares = false;
    erasingKicks = false;
    erasingPerc1 = false;
    erasingPerc2 = false;
    erasingSmp1 = false;
    erasingSmp2 = false;
    erasingHits = false;
    erasingAll = false;
    copyToActiveGroup(obj);
  }
}
void readBtn_Accent(Bar & obj) {
  if ( digitalRead(accentSelectPin) == LOW && !accentPushedRN) {    // accent button
    accentPushedRN = 1;
    accent = true;
    accentVolLastIntermediate = analogRead(trckVolPn);      //read when accent button is pushed, so can update only when the knob moves 
  }
  else if (digitalRead(accentSelectPin) == HIGH && accentPushedRN) {
    accentPushedRN = 0;
    accent = false;
    copyFromActiveGroup(obj);
    updateLCD(1,tempoBPM,1);      // update all info on lcd (return to normal)
  }
}
void readBtn_TrackVol(Bar & obj) {
  if ( digitalRead(trackVolBtnPin) == LOW && !trackVolBtnPushedRN ) {
    trackVolBtnPushedRN = 1;
  } else if (digitalRead(trackVolBtnPin) == HIGH && trackVolBtnPushedRN) {
    trackVolBtnPushedRN = 0;
  }
}
void readBtn_Poly(Bar & obj) {
  if ( digitalRead(polyBtnPin) == LOW && !polyBtnPushedRN ) {
    polyBtnPushedRN = 1;
    poly = true;
  } else if (digitalRead(polyBtnPin) == HIGH && polyBtnPushedRN) {
    polyBtnPushedRN = 0;
    poly = false;
  }
}

void checkPolyButtons() {
  Serial.println("checkPoly");
  if (!polyBtnPshdRN) {
    for (int i = 0; i < 16 ; i++)
      if ( digitalRead(buttonPin[i]) == LOW ) {
        selectedLastBeat = i;
        polyFlag = true; // set the latch to set the poly at beat 0 for the selected group
        writePolyLEDs();
        polyBtnPshdRN = 1;
        lastPolyBtnPushed = i;
      }
  }
  else if ( digitalRead(buttonPin[lastPolyBtnPushed]) == HIGH) {
    polyBtnPshdRN = 0;
  }
}
void checkBeatButtons(Bar & obj) {
  if (!beatBtnPshdRN) {
    for (int i = 0; i < 16 ; i++) {
      if (sampleSelectPushedRN && digitalRead(buttonPin[i]) == LOW) {   //just plays the samples btn 1 plays sample1, 2 plays 2 etc, for finding samples
        if (i < 16) {
          wTrig.trackPlayPoly( getLiveSample(i) );
          selectionMem[currentGroup] = i;
          newSampleSelected = true;
          updateLCD(true, 0, false);
        }
        beatBtnPshdRN = 1;
        lastBtnPushed = i;
      }
      else if (erasePushedRN && digitalRead(buttonPin[i]) == LOW) {  //add pattern sequence
        if (i == 0) {
          erasingHats = true;
          Serial.println("Erasing Hats");
        } else if (i == 1) {
          erasingSnares = true;
          Serial.println("Erasing Snares");
        } else if (i == 2) {
          erasingKicks = true;
          Serial.println("Erasing Kicks");
        } else if (i == 3) {
          erasingPerc1 = true;
          Serial.println("Erasing Perc1");
        } else if (i == 4) {
          erasingPerc2 = true;
          Serial.println("Erasing Perc2");
        } else if (i == 5) {
          erasingSmp1 = true;
          Serial.println("Erasing Smp1");
        } else if (i == 6) {
          erasingSmp2 = true;
          Serial.println("Erasing Smp2");
        } else if (i == 7) {
          erasingHits = true;
          Serial.println("Erasing Hits");
        } else if (i == 9) {
          erasingAll = true;
          Serial.println("Erasing Hats");
        }
      }
      else if (functionBtnPushedRN && digitalRead(buttonPin[i]) == LOW) {   //function button methods
        if (i == 0) {
          if (!copyPatternFlag)
            addPattern(1);
          else
            copy(1);
          Serial.print("add pat A");
          beatBtnPshdRN = 1;
          lastBtnPushed = i;
        }
        else if (i == 1) {
          if (!copyPatternFlag)
            addPattern(2);
          else
            copy(2);
          Serial.print("add pat B");
          beatBtnPshdRN = 1;
          lastBtnPushed = i;
        }
        else if (i == 2) {
          if (!copyPatternFlag)
            addPattern(3);
          else
            copy(3);
          Serial.print("add pat C");
          beatBtnPshdRN = 1;
          lastBtnPushed = i;
        }
        else if (i == 3) {
          if (!copyPatternFlag)
            addPattern(4);
          else
            copy(4);
          Serial.print("add pat F");
          beatBtnPshdRN = 1;
          lastBtnPushed = i;
        }
        else if (i == 4) {
          Serial.print("Copy Pattern");
          copyPatternFlag = true;
          writePatternLEDs();
          beatBtnPshdRN = 1;
          lastBtnPushed = i;
        }
        if (i == 5) {}//randomizeContinuously(obj);
        if (i == 6) {}
        if (i == 7) {}//fillFlag = true;
        if (i == 8) {
          Serial.println("randomize");
          randomize(obj);
        }
        if (i == 9) {
          allTripletFlag = true;
          tripletFlag = true;
        }
        else if (i == 10) tripletFlag = true;
        else if (i == 11) {
          liveMash = liveMash ? false : true;
          if (liveMash == false) displayingLiveMash = false; // helper so the lcd only updates once when live is turned on
          updateLCD(true, 0, false); //update lcd to toggle display "LIVE"
          Serial.print("LIVE MASH PUSHED!  ");
          Serial.println(liveMash);
        }
        else if (i == 12) setDelay();
        else if (i == 13) lockedSamples = lockedSamples ? false : true;
        else if (i == 14) save(obj);
        else if (i == 15) load(obj);
        beatBtnPshdRN = 1;
        lastBtnPushed = i;
      }
      else if (liveMash && digitalRead(buttonPin[i]) == LOW) { // Live mode
        wTrig.trackPlayPoly( livePlaySamples[i] );
        beatBtnPshdRN = 1;
        lastBtnPushed = i;
      }
      else if ( digitalRead(buttonPin[i]) == LOW) {  // add current sample to selected beat
        if ( buttonsActive[i] == 0) {
          buttonsActive[i] = 1;
          addSample(i + 1, obj);             //adds sample to corresponding beat
          writeBeatLEDs();
        }
        else {
          buttonsActive[i] = 0;
          removeSample(i + 1, obj);
          //removeRollSample(i, obj);
          writeBeatLEDs();
        }
        beatBtnPshdRN = 1;
        lastBtnPushed = i;
      }
    }
  }
  else if ( digitalRead(buttonPin[lastBtnPushed]) == HIGH) {
    beatBtnPshdRN = 0;
    //Serial.println("RESET BUTTONPUSHED");
  }
}

void checkRollButtons(Bar & obj) {
  if (!rollBtnPshdRN) {
    for (int i = 0; i < 16 ; i++) {
      if ( digitalRead(buttonPin[i]) == LOW ) {  // correspond to digital inputs 2-5
        if (buttonsActive[i] == 1 && rollBtnsActive[i] == 0) {
          rollBtnsActive[i] = 1;
          //removeSample(i + 1, obj);
          addRollSample(i, obj);
          writeRollLEDs();
        }
        else if (buttonsActive[i] == 1 && rollBtnsActive[i] == 1) {
          rollBtnsActive[i] = 0;
          //addSample(i + 1, obj);
          removeRollSample(i, obj);
          writeRollLEDs();
        }
        else if (buttonsActive[i] == 0 && rollBtnsActive[i] == 1) {
          //rollBtnsActive[i] = 0;
          //removeRollSample(i, obj);
          //writeRollLEDs();
        }
        rollBtnPshdRN = 1;
        lastRollBtnPushed = i;
      }
    }
  }
  else if ( digitalRead(buttonPin[lastRollBtnPushed]) == HIGH) {
    rollBtnPshdRN = 0;
  }
}


void checkAccentButtons(Bar & obj) {
  if (!accentBtnPshdRN) {
    for (int i = 0; i < 16 ; i++) {
      if ( digitalRead(buttonPin[i]) == LOW ) {  // correspond to digital inputs 2-5
        if ( accentBtnsActive[i] == 0) {  // button has been pushed and was low before
          accentBtnsActive[i] = 1;
          writeAccentLEDs();
          if (trackVolumes[getCrrntSlctdSmpl()] == 0) {   // lowers total vol when add accent so its not overpowering
            wTrig.trackGain(getCrrntSlctdSmpl(), -10);
            Serial.println (getCrrntSlctdSmpl()); Serial.print(" Set to -10Db");
            trackVolumes[getCrrntSlctdSmpl()] = -10;
          }
        }
        else {
          accentBtnsActive[i] = 0;
          writeAccentLEDs();
        }
        accentBtnPshdRN = 1;
        lastAccentBtnPushed = i;
      }
    }
  }
  else if ( digitalRead(buttonPin[lastAccentBtnPushed]) == HIGH) {
    accentBtnPshdRN = 0;
  }
}
//-*******************************************************************************- LEDS -********************************************************************************************************-//

void writePolyLEDs() {
  int b;
  byte ledByte1 = 0;
  byte ledByte2 = 0;
  if (hatGroupSelect) b = lastHatBeat;
  else if (snareGroupSelect) b = lastSnareBeat;
  else if (kickGroupSelect) b = lastKickBeat;
  else if (perc1GroupSelect) b = lastPerc1Beat;
  else if (perc2GroupSelect) b = lastPerc2Beat;
  else if (sample1GroupSelect) b = lastSmp1Beat;
  else if (sample2GroupSelect) b = lastSmp2Beat;
  else if (hitGroupSelect) b = lastHitBeat;

  if (b <= 8) ledByte1 = 1 << b - 1;
  else ledByte2 = 1 << b - 9;
  beatLeds(ledByte1, ledByte2);
  Serial.print("lastBeat:"); Serial.print(b); Serial.print(" bytes:"); Serial.print(ledByte1); Serial.println(ledByte2);
}

void writePatternLEDs() {
  int b = chain[currentSlotPtr];
  byte ledByte1 = 0;
  byte ledByte2 = 0;

  if (copyPatternFlag) ledByte1 = 16;
  //if (copyPtr >= 2)ledByte1 = 248;

  if (b <= 8) ledByte1 |= 1 << b - 1 ;
  else ledByte2 = 0;
  beatLeds(ledByte1, ledByte2);
}

void writeBeatLEDs() {
  byte ledByteA = 0;
  byte ledByteB = 0;
  for (int i = 0; i < 8; i++) {   // convert the array of active buttons into byte format - [0][1][7] && [9] are high it looks like 11000001(A) 01000000(B)
    if (buttonsActive[i])
      ledByteA |= 1 << (i);
    if (buttonsActive[i + 8])
      ledByteB |= 1 << (i);
  }
  if (!pauseOn) {
    int selBeat = selectedBeat(); // the beat is always incremented right after being played so the true last beat played is -1
    if ( selBeat < 8 )
      ledByteA |= 1 << (selBeat);
    else
      ledByteB |= 1 << (selBeat - 8);
  }
  beatLeds(ledByteA, ledByteB);
}

void writeAccentLEDs() {
  byte ledByteA = 0;
  byte ledByteB = 0;
  for (int i = 0; i < 8; i++) {   // convert the array of active buttons into byte format - [0][1][7] && [9] are high it looks like 11000001(A) 01000000(B)
    if (accentBtnsActive[i])
      ledByteA |= 1 << (i);
    if (accentBtnsActive[i + 8])
      ledByteB |= 1 << (i);
  }
  beatLeds(ledByteA, ledByteB);
}

void writeRollLEDs() {
  byte ledByteA = 0;
  byte ledByteB = 0;
  for (int i = 0; i < 8; i++) {   // convert the array of active buttons into byte format - [0][1][7] && [9] are high it looks like 11000001(A) 01000000(B)
    if (rollBtnsActive[i])
      ledByteA |= 1 << (i);
    if (rollBtnsActive[i + 8])
      ledByteB |= 1 << (i);
  }
  beatLeds(ledByteA, ledByteB);
}

void animate(int selection, bool save) {
  byte b = 0;
  byte a = 0;
  if (selection < 8) {
    b |= 1 << selection;
    beatLeds(b, 0); delay(400);
    beatLeds(0, 0); delay(400);
    beatLeds(b, 0); delay(400);
    beatLeds(0, 0); delay(400);
  } else {
    a = 1 << selection - 8;
    beatLeds(0, a); delay(400);
    beatLeds(0, 0); delay(400);
    beatLeds(0, a); delay(400);
    beatLeds(0, 0); delay(400);
  }
  if (save) {
    for (int i = 0 ; i < 8 ; i++) {
      b = 1 << i;
      a = 128 >> i;
      beatLeds(b, a);
      delay(40);
    }
    for (int i = 0 ; i < 8 ; i++) {
      b = 1 << i;
      a = 128 >> i;
      beatLeds(b, a);
      delay(40);
    }
  } else {
    for (int i = 0 ; i < 8 ; i++) {
      b |= 128 >> i;
      a |= 1 << i;
      beatLeds(b, a);
      delay(80);
    }
    for (int i = 0 ; i < 8 ; i++) {
      b &= 254 << i;
      a &= 127 >> i;
      beatLeds(b, a);
      delay(80);
    }
  }
  beatLeds(0, 0);
}


void updateGroupData() {
  groupLeds[0] = hatGroupSelect;
  groupLeds[1] = snareGroupSelect;
  groupLeds[2] = kickGroupSelect;
  groupLeds[3] = perc1GroupSelect;
  groupLeds[4] = perc2GroupSelect;
  groupLeds[5] = sample1GroupSelect;
  groupLeds[6] = sample2GroupSelect;
  groupLeds[7] = hitGroupSelect;
  for ( int i = 0 ; i < 8 ; i++)
    if ( groupLeds[i])
      groupLedData = 1 << i;
}

void beatLeds(byte dataA, byte dataB) {
  //  for ( int i = 0 ; i < 2 ; i++ )
  //    byteData[i] = data >> i*8;
  digitalWrite(latchPin, 0);      //ground latchPin and hold low for as long as you are transmitting
  shiftOut(dataPin, clockPin, groupLedData);    //move 'em out
  shiftOut(dataPin, clockPin, dataA);    //move 'em out
  shiftOut(dataPin, clockPin, dataB);    //move 'em out
  digitalWrite(latchPin, 1);  //return the latch pin high to signal chip that it   //no longer needs to listen for information
}

void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first,//on the rising edge of the clock,//clock idles low//internal function setup
  int i = 0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);
  //clear everything out just in case to//prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);
  //for each bit in the byte myDataOut�//NOTICE THAT WE ARE COUNTING DOWN in our for loop//This means that %00000001 or "1" will go through such//that it will be pin Q0 that lights.
  for (i = 7; i >= 0; i--)  {
    digitalWrite(myClockPin, 0);
    //if the value passed to myDataOut and a bitmask result// true then... so if we are at i=6 and our value is// %11010100 it would the code compares it to %01000000
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1 << i) ) {
      pinState = 1;
    }
    else {
      pinState = 0;
    }
    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin
    digitalWrite(myClockPin, 1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
  }
  //stop shifting
  digitalWrite(myClockPin, 0);
}
void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);

  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);

  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  Serial.print("R = "); Serial.print(r, DEC);
  Serial.print(" G = "); Serial.print(g, DEC);
  Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}
