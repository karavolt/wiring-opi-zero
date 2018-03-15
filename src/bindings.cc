#include <node.h>
#include <v8.h>

#include <wiringPi.h>
#include <drcSerial.h>
#include <max5322.h>
#include <max31855.h>
#include <mcp23s08.h>
#include <mcp23s17.h>
#include <mcp3002.h>
#include <mcp3004.h>
#include <mcp3422.h>
#include <mcp4802.h>
#include <mcp23008.h>
#include <mcp23016.h>
#include <mcp23017.h>
#include <pcf8574.h>
#include <pcf8591.h>
#include <sn3218.h>
//#include <pca9685.h>
#include <softPwm.h>
#include <softServo.h>
//#include <softTone.h>
#include <sr595.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>
#include <wiringShift.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdio.h>

using namespace v8;

// We don't want to include the whole node headers :)
namespace node {
  namespace Buffer {
    bool HasInstance(v8::Handle<v8::Value> val);
    bool HasInstance(v8::Handle<v8::Object> val);
    char* Data(v8::Handle<v8::Value> val);
    char* Data(v8::Handle<v8::Object> val);
    size_t Length(v8::Handle<v8::Value> val);
    size_t Length(v8::Handle<v8::Object> val);
  }
}

bool find_string(const char* string, const char* array[], size_t s) {
  for (size_t i = 0; i < s; i++) {
    if (!strcasecmp(string, array[i])) {
      return true;
    }
  }
  return false;
}

bool find_int(const int value, const int array[], size_t s) {
  for (size_t i = 0; i < s; i++) {
    if (value == array[i]) {
      return true;
    }
  }
  return false;
}


#if NODE_VERSION_AT_LEAST(0, 11, 0)
void throw_error(Isolate* isolate, const char* format, ...) {
#else
void throw_error(const char* format, ...) {
#endif
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, 156, format, args);
  va_end(args);
  
  #if NODE_VERSION_AT_LEAST(0, 11, 0)
    isolate->ThrowException(Exception::Error(String::NewFromUtf8(isolate, buffer)));
  #else
    ThrowException(Exception::Error(String::New(buffer)));
  #endif
}



#if NODE_VERSION_AT_LEAST(0, 11, 0)

	#define DECLARE(name) \
	  void name(const FunctionCallbackInfo<Value>& args);
    
	#define IMPLEMENT(name) \
	  void name(const FunctionCallbackInfo<Value>& args)
  
	#define EXPORT_FUNCTION(NAME) \
		NODE_SET_METHOD(exports, #NAME, NAME)
      
  #define EXPORT_CONSTANT_INT(name) \
    exports->ForceSet(String::NewFromUtf8(isolate, #name, String::kInternalizedString), \
      Int32::New(isolate, name), static_cast<PropertyAttribute>(ReadOnly | DontDelete));
  
  #define EXPORT_CONSTANT_STRING(name) \
    exports->ForceSet(String::NewFrontUtf8(isolate, #name, String::kInternalizedString), \
      String::NewFromUtf8(isolate, name), static_cast<PropertyAttribute>(ReadOnly | DontDelete));
  
  #define EXPORT_CONSTANT_INT_ARRAY(name, array, length) \
    { \
      Local<Array> arr = Array::New(isolate, length); \
      for (int i = 0; i < length; i++) { \
        arr->Set(i, Int32::New(isolate, array[i])); \
      } \
      target->ForceSet(String::NewFromUtf8(isolate, #name, String::kInternalizedString), \
        arr, static_cast<PropertyAttribute>(ReadOnly | DontDelete)); \
    }
    
  #define EXPORT_CONSTANT_STRING_ARRAY(name, array, length) \
    { \
      Local<Array> arr = Array::New(isolate, length); \
      for (int i = 0; i < length; i++) { \
        arr->Set(i, String::NewFromUtf8(isolate, array[i])); \
      } \
      target->ForceSet(String::NewFromUtf8(isolate, #name, String::kInternalizedString), \
        arr, static_cast<PropertyAttribute>(ReadOnly | DontDelete)); \
    }

  #define NODE_MODULE_INIT() \
    namespace nodemodule { \
      void init(Handle<Object> target); \
    } \
    void nodemodule::init(Handle<Object> target)
  
  #define NODE_MODULE_DECLARE(name) NODE_MODULE(name, nodemodule::init)
  #define IMPLEMENT_EXPORT_INIT(name) void nodemodule::init##name(Isolate* isolate, Handle<Object> target)
  #define DECLARE_EXPORT_INIT(name) \
    namespace nodemodule { \
      void init##name(Isolate* isolate, Handle<Object> target); \
    }
  
  #define INIT(name) nodemodule::init##name(isolate, target);
  
	#define SCOPE_OPEN() \
	  Isolate* isolate = args.GetIsolate(); \
	  	HandleScope scope(isolate)
	#define SCOPE_CLOSE(obj) args.GetReturnValue().Set(obj)

#else

  #define DECLARE(name) \
    namespace nodemodule { \
      static Handle<Value> name(const Arguments& args); \
    }
    
  #define IMPLEMENT(name) \
    Handle<Value> nodemodule::name(const Arguments& args)
  
  #define EXPORT_FUNCTION(name)  \
    target->Set(String::NewSymbol(#name), \
      FunctionTemplate::New(nodemodule::name)->GetFunction())
    
  #define EXPORT_CONSTANT_INT(name) \
    target->Set(String::NewSymbol(#name), \
      Int32::New(name), static_cast<PropertyAttribute>(ReadOnly | DontDelete));
  
  #define EXPORT_CONSTANT_STRING(name) \
    target->Set(String::NewSymbol(#name), \
      String::New(name), static_cast<PropertyAttribute>(ReadOnly | DontDelete));
    
  #define EXPORT_CONSTANT_INT_ARRAY(name, array, length) \
    { \
      Local<Array> arr = Array::New(length); \
      for (int i = 0; i < length; i++) { \
        arr->Set(i, Int32::New(array[i])); \
      } \
      target->Set(String::NewSymbol(#name), arr, static_cast<PropertyAttribute>(ReadOnly | DontDelete)); \
    }
    
  #define EXPORT_CONSTANT_STRING_ARRAY(name, array, length) \
    { \
      Local<Array> arr = Array::New(length); \
      for (int i = 0; i < length; i++) { \
        arr->Set(i, String::New(array[i])); \
      } \
      target->Set(String::NewSymbol(#name), arr, static_cast<PropertyAttribute>(ReadOnly | DontDelete)); \
    }
  
  #define NODE_MODULE_INIT() \
    namespace nodemodule { \
      void init(Handle<Object> target); \
    } \
    void nodemodule::init(Handle<Object> target)
  #define NODE_MODULE_DECLARE(name) NODE_MODULE(name, nodemodule::init)
  #define IMPLEMENT_EXPORT_INIT(name) void nodemodule::init##name(Handle<Object> target)
  #define DECLARE_EXPORT_INIT(name) \
    namespace nodemodule { \
      void init##name(Handle<Object> target); \
    }
  
  #define INIT(name) nodemodule::init##name(target);
  
  #define SCOPE_OPEN() HandleScope scope
  #define SCOPE_CLOSE(obj) return scope.Close(obj)

#endif



#if NODE_VERSION_AT_LEAST(0, 11, 0)
  #define UNDEFINED() Undefined(isolate)
  #define INT32(v) Int32::New(isolate, v)
  #define UINT32(v) Uint32::NewFromUnsigned(isolate, v)
  #define STRING(v) String::NewFromUtf8(isolate, v)
#else
  #define UNDEFINED() Undefined()
  #define INT32(v) Int32::New(v)
  #define UINT32(v) Uint32::NewFromUnsigned(v)
  #define STRING(v) String::New(v)
#endif

#if NODE_VERSION_AT_LEAST(0, 11, 0)
  #define THROW_ERROR(fmt, ...) \
    throw_error(isolate, fmt, __VA_ARGS__); \
    SCOPE_CLOSE(UNDEFINED());
#else
  #define THROW_ERROR(fmt, ...) \
    throw_error(fmt, __VA_ARGS__); \
    SCOPE_CLOSE(UNDEFINED())
#endif

#define SET_ARGUMENT_NAME(id, name) static const char* arg##id = #name
#define GET_ARGUMENT_NAME(id) arg##id

#define CHECK_ARGUMENTS_LENGTH_EQUAL(length) \
  if (args.Length() != length) { \
    THROW_ERROR("%s: arguments.length => (%i === %i) === false", __func__, args.Length(), length); \
  }
  
#define CHECK_ARGUMENT_TYPE(id, istype) \
  if (!args[id]->istype()) { \
    THROW_ERROR("%s: %s(arguments['%s']) === false", __func__, #istype, GET_ARGUMENT_NAME(id)); \
  }

#define CHECK_ARGUMENT_TYPE_ARRAY(id) CHECK_ARGUMENT_TYPE(id, IsArray)
#define CHECK_ARGUMENT_TYPE_INT32(id) CHECK_ARGUMENT_TYPE(id, IsInt32)
#define CHECK_ARGUMENT_TYPE_UINT32(id) CHECK_ARGUMENT_TYPE(id, IsUint32)
#define CHECK_ARGUMENT_TYPE_NUMBER(id) CHECK_ARGUMENT_TYPE(id, IsNumber)
#define CHECK_ARGUMENT_TYPE_STRING(id) CHECK_ARGUMENT_TYPE(id, IsString)
#define CHECK_ARGUMENT_TYPE_FUNCTION(id) CHECK_ARGUMENT_TYPE(id, IsFunction)
#define CHECK_ARGUMENT_TYPE_OBJECT(id) CHECK_ARGUMENT_TYPE(id, IsObject)
#define CHECK_ARGUMENT_TYPE_NODE_BUFFER(id) \
  if (!(args[id]->IsObject() && node::Buffer::HasInstance(args[id]))) { \
    THROW_ERROR("%s: %s(arguments['%s']) === false", __func__, "isBuffer", GET_ARGUMENT_NAME(id)); \
  }
  
#define CHECK_ARGUMENT_ARRAY_LENGTH(id, length) \
  if (!(Local<Array>::Cast(args[id])->Length() == length)) { \
    THROW_ERROR("%s: (arguments['%s'].length === %i) === false", __func__, GET_ARGUMENT_NAME(id), length); \
  }

#define GET_ARGUMENT_AS_TYPE(id, type) args[id]->type()

#define GET_ARGUMENT_AS_INT32(id) GET_ARGUMENT_AS_TYPE(id, Int32Value)
#define GET_ARGUMENT_AS_UINT32(id) GET_ARGUMENT_AS_TYPE(id, Uint32Value)
#define GET_ARGUMENT_AS_NUMBER(id) GET_ARGUMENT_AS_TYPE(id, NumberValue)
#define GET_ARGUMENT_AS_STRING(id) GET_ARGUMENT_AS_TYPE(id, ToString)
#define GET_ARGUMENT_AS_LOCAL_FUNCTION(id) Local<Function>::Cast(args[id])

#if !NODE_VERSION_AT_LEAST(0, 11, 0)
  #define GET_ARGUMENT_AS_PERSISTENT_FUNCTION(id) Persistent<Function>::New(GET_ARGUMENT_AS_LOCAL_FUNCTION(id))
#endif

#define LIST(...) { __VA_ARGS__ }
#define CHECK_ARGUMENT_IN_STRINGS(id, value, T) \
  { \
    static const char* strings[] = LIST T; \
    if (!find_string(*value, strings, sizeof(strings) / sizeof(char*))) { \
      THROW_ERROR("%s: arguments['%s'] => (\"%s\" in %s) === false", __func__, GET_ARGUMENT_NAME(id), *value, #T); \
    } \
  }
  
#define CHECK_ARGUMENT_IN_INTS(id, value, T) \
  { \
    static const int ints[] = LIST T; \
    if (!find_int(value, ints, sizeof(ints) / sizeof(int))) { \
      THROW_ERROR("%s: arguments['%s'] => (%i in %s) === false", __func__, GET_ARGUMENT_NAME(id), value, #T); \
    } \
  }
  
#define CHECK_ARGUMENT_IN_RANGE(id, value, min, max) \
  if (value < min || value > max) { \
    THROW_ERROR("%s: arguments['%s'] => inRange(%i, [%i, %i]) === false", __func__, GET_ARGUMENT_NAME(id), value, min, max); \
  }


#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

namespace wpi {
	
	// test
	DECLARE(world);
	
	//DECLARE(setup);
	DECLARE(wiringPiSetup);
	DECLARE(wiringPiSetupGpio);
	DECLARE(wiringPiSetupSys);
	DECLARE(wiringPiSetupPhys);

	// Core functions
	DECLARE(pinModeAlt);
	DECLARE(pinMode);
	DECLARE(pullUpDnControl);
	DECLARE(digitalRead);
	DECLARE(digitalWrite);
	DECLARE(pwmWrite);
	DECLARE(analogRead);
	DECLARE(analogWrite);
	//DECLARE(pulseIn);

	DECLARE(delay);
	DECLARE(delayMicroseconds);
	DECLARE(millis);
	DECLARE(micros);

	// On-Board Rasberry Pi hardware specific stuff
	DECLARE(piBoardRev);
	DECLARE(piBoardId);
	DECLARE(wpiPinToGpio);
	DECLARE(physPinToGpio);
	DECLARE(setPadDrive);
	DECLARE(getAlt);
	DECLARE(pwmToneWrite);
	DECLARE(digitalWriteByte);
	DECLARE(pwmSetMode);
	DECLARE(pwmSetRange);
	DECLARE(pwmSetClock);
	DECLARE(gpioClockSet);

	// Extensions
	DECLARE(drcSetupSerial);
	DECLARE(max5322Setup);
	DECLARE(max31855Setup);
	DECLARE(mcp23s08Setup);
	DECLARE(mcp23s17Setup);
	DECLARE(mcp3002Setup);
	DECLARE(mcp3004Setup);
	DECLARE(mcp3422Setup);
	DECLARE(mcp4802Setup);
	DECLARE(mcp23008Setup);
	DECLARE(mcp23016Setup);
	DECLARE(mcp23017Setup);
	DECLARE(pcf8574Setup);
	DECLARE(pcf8591Setup);
	DECLARE(sn3218Setup);
	DECLARE(sr595Setup);
	//  DECLARE(pca9685Setup);

	// Soft PWM
	DECLARE(softPwmCreate);
	DECLARE(softPwmWrite);
	DECLARE(softPwmStop);

	// Soft Servo
	DECLARE(softServoWrite);
	DECLARE(softServoSetup);

	// Soft Tone
	//  DECLARE(softToneCreate);
	//  DECLARE(softToneWrite);
	//  DECLARE(softToneStop);

	// WiringPI I2C
	DECLARE(wiringPiI2CRead);
	DECLARE(wiringPiI2CReadReg8);
	DECLARE(wiringPiI2CReadReg16);
	DECLARE(wiringPiI2CWrite);
	DECLARE(wiringPiI2CWriteReg8);
	DECLARE(wiringPiI2CWriteReg16);
	DECLARE(wiringPiI2CSetupInterface);
	DECLARE(wiringPiI2CSetup);
	DECLARE(wiringPiI2CClose)

	// WiringPI SPI
	DECLARE(wiringPiSPIGetFd);
	DECLARE(wiringPiSPIDataRW);
	DECLARE(wiringPiSPISetup);
	DECLARE(wiringPiSPISetupMode);
	DECLARE(wiringPiSPIClose);

	// WiringPi Serial
	DECLARE(serialOpen);
	DECLARE(serialClose);
	DECLARE(serialFlush);
	DECLARE(serialPutchar);
	DECLARE(serialPuts);
	DECLARE(serialPrintf);
	DECLARE(serialDataAvail);
	DECLARE(serialGetchar);

	// WiringPi Shift
	DECLARE(shiftIn);
	DECLARE(shiftOut);
	

	//void setup(const FunctionCallbackInfo<Value>& args) {
	IMPLEMENT(world) {
	  //Isolate* isolate = args.GetIsolate();
	  SCOPE_OPEN();
	  
	  printf("%s() at %s::%05d\n", __func__, __FILE__, __LINE__);
	
	  //#define SCOPE_CLOSE(obj) args.GetReturnValue().Set(obj)  
	  SCOPE_CLOSE(STRING("world !!"));
	}
	
	// === Setup ===
	
	/*
	IMPLEMENT(setup) {
	  SCOPE_OPEN();

		char *mode = "wpi";
	  
		if (args.Length() > 0) { 
			SET_ARGUMENT_NAME(0, mode2);
		
		  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

		  CHECK_ARGUMENT_TYPE_STRING(0);

		  #if NODE_VERSION_AT_LEAST(0, 11, 0)
		    String::Utf8Value mode2(GET_ARGUMENT_AS_STRING(0));
		  #else
		    String::AsciiValue mode2(GET_ARGUMENT_AS_STRING(0));
		  #endif
			
		  CHECK_ARGUMENT_IN_STRINGS(0, mode2, ("wpi", "gpio", "sys", "phys"));
		  
		  strcpy(mode, *mode2);
		}
		
		printf("setup() / mode = %s\n" , mode);

	  int res = 0;
	  if (!strcasecmp(mode, "wpi")) {
	    res = ::wiringPiSetup();
	  }
	  else if (!strcasecmp(mode, "gpio")) {
	    res = ::wiringPiSetupGpio();
	  }
	  else if (!strcasecmp(mode, "sys")) {
	    res = ::wiringPiSetupSys();
	  }
	  else if (!strcasecmp(mode, "phys")) {
	    res = ::wiringPiSetupPhys();
	  }

	  // libWiringPi v2 setup functions always returns 0, so this check is kind of useless, unless v1 behaviour is restored
	  // NOTE: If you want to restore the v1 behaviour, then you need to set the
	  // environment variable: WIRINGPI_CODES (to any value, it just needs to exist)
	  SCOPE_CLOSE(INT32(res));
	}
	*/

	// Func : int wiringPiSetup(void)
	// Returns : error code if v1 mode otherwise always returns 0
	// Description : Initialises wiringPi and assumes that the calling program is going
	// to be using the wiringPi pin numbering scheme.
	// This is a simplified numbering scheme which provides a mapping from virtual
	// pin numbers 0 through 16 to the real underlying Broadcom GPIO pin numbers.
	// see the pins page (https://projects.drogon.net/raspberry-pi/wiringpi/pins/) for a table
	// which maps the wiringPi pin number to the Broadcom GPIO pin number to the physical location
	// on the edge connector.
	// This function needs to be called with root privileges.

	IMPLEMENT(wiringPiSetup) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  int res = ::wiringPiSetup();
	  	
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiSetupGpio(void)
	// Returns : error code if v1 mode otherwise always returns 0
	// Description : This is indential to above, however it allows the calling programs to use
	// the Broadcom GPIO pin numbers directly with no re-mapping.
	// As above, this function needs to be called with root privileges, and note that some pins
	// are different from revision 1 to revision 2 boards.

	IMPLEMENT(wiringPiSetupGpio) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  int res = ::wiringPiSetupGpio();

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiSetupSys(void)
	// Returns : error code if v1 mode otherwise always returns 0
	// Description : This initialises wiringPi but uses the /sys/class/gpio interface rather than
	// accessing the hardware directly. This can be called as a non-root user provided the GPIO pins
	// have been exported before-hand using gpio program. Pin numbering in this mode is the native
	// Broadcom GPIO numbers - the same as wiringPiSetGpio above, so be aware of the differences
	// between Rev 1 and Rev 2 boards.
	// Note: In this mode you can only use the pins which have been exported via the
	// /sys/class/gpio interface before you run your program. You can do this in a seperate
	// shell script, or by using the system() function from inside your program to call the gpio program.
	// Also note that some functions have no effect when using this mode as they're not currently
	// possible to action unless called with root privileges. (although you can use system() to call
	// gpio to set/change modes if needed).

	IMPLEMENT(wiringPiSetupSys) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  int res = ::wiringPiSetupSys();

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiSetupPhys(void)
	// Returns : error code if v1 mode otherwise always returns 0
	// Description : Identical to above, however it allows the calling programs to use
	// the physical pin numbers on the P1 connector only.
	// As above, this function needs to be called with root priviliges.

	IMPLEMENT(wiringPiSetupPhys) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  int res = ::wiringPiSetupPhys();

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void pinModeAlt(int pin, int mode)
	// Description : This is an un-documented special to let you set any pin to any mode.
	// Modes are FSEL_INPT, FSEL_OUTP, FSEL_ALT0, FSEL_ALT1, FSEL_ALT2, FSEL_ALT3, FSEL_ALT4, FSEL_ALT5.

	IMPLEMENT(pinModeAlt) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, mode);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int mode = GET_ARGUMENT_AS_INT32(1);

	  CHECK_ARGUMENT_IN_INTS(1, mode, (FSEL_INPT, FSEL_OUTP, FSEL_ALT0, FSEL_ALT1, FSEL_ALT2, FSEL_ALT3, FSEL_ALT4, FSEL_ALT5));

	  ::pinModeAlt(pin, mode);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pinMode(int pin, int mode)
	// Description : This sets the mode of a pin to either INPUT, OUTPUT, PWM_OUTPUT or GPIO_CLOCK.
	// Note that only wiringPi pin 1 (BCM_GPIO 18) supports PWM output and only wiringPi pin 7 (BCM_GPIO 4)
	// supports CLOCK output modes.
	// This function has no effect when in Sys mode. If you need to change the pin mode, the you can
	// do it with the gpio program in a script before you start your program.

	IMPLEMENT(pinMode) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, mode);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int mode = GET_ARGUMENT_AS_INT32(1);

	  CHECK_ARGUMENT_IN_INTS(1, mode, (INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK, SOFT_PWM_OUTPUT, SOFT_TONE_OUTPUT));
	  
	  ::pinMode(pin, mode);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pullUpDnControl(int pin, int pud)
	// Description : This sets the pull-up or pull-down resistor mode on the given pin, which should be set
	// as an input. Unlike Arduino, the BCM2835 has both pull-up and down internal resistors.
	// The parameter pud should be; PUD_OFF (no pull up/down), PUD_DOWN (pull to ground) or PUD_UP (pull to 3.3v).
	// The internal pull up/down resistors have a value of approximately 50KΩ on the Raspberry Pi.

	IMPLEMENT(pullUpDnControl) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, pud);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int pud = GET_ARGUMENT_AS_INT32(1);

	  CHECK_ARGUMENT_IN_INTS(1, pud, (PUD_OFF, PUD_DOWN, PUD_UP));

	  ::pullUpDnControl(pin, pud);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : int digitalRead(int pin)
	// Description : This function returns the value read at the given pin. It will be HIGH or LOW (1 or 0)
	// depending on the logic level at the pin.

	IMPLEMENT(digitalRead) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int res = ::digitalRead(pin);

	  // Make sure the function returns strictly 1 or 0
	  // §4.7/4 from the C++ Standard says (Integral Conversion)
	  // If the source type is bool, the value false is converted to zero and the value true is converted to one.
	  res = (res != 0);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void digitalWrite(int pin, int value)
	// Description : Write the value HIGH or LOW (1 or 0) to the given pin which must have been
	// previously set as an output.
	// WiringPi treats any non-zero number as HIGH, however 0 is the only representation of LOW.

	IMPLEMENT(digitalWrite) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, state);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int state = GET_ARGUMENT_AS_INT32(1);

	  CHECK_ARGUMENT_IN_INTS(1, state, (HIGH, LOW));
	  
	  ::digitalWrite(pin, state);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pwmWrite(int pin, int value)
	// Description : Writes the value to the PWM register for the given pin. The Raspberry Pi has
	// one on-board PWM pin, pin 1 (BCM_GPIO 18, Phys 12) and the range is 0-1024. Other PWM
	// devices may have other PWM ranges.
	// This function is not able to control the Pi's on-board PWM when in Sys mode.

	IMPLEMENT(pwmWrite) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, value);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);

	  ::pwmWrite(pin, value);

	  SCOPE_CLOSE(UNDEFINED());
	}

	IMPLEMENT(analogRead) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int res = ::analogRead(pin);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void analogWrite(int pin, int value)
	// Description : This writes the given value to the supplied analog pin. You will need to register
	// additional analog modules to enable this function for devices such as the Gertboard.

	IMPLEMENT(analogWrite) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, value);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);

	  ::analogWrite(pin, value);

	  SCOPE_CLOSE(UNDEFINED());
	}

	/*
	IMPLEMENT(pulseIn) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, state);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int state = GET_ARGUMENT_AS_INT32(1);

	  CHECK_ARGUMENT_IN_INTS(1, state, (HIGH, LOW));

	  int us = ::pulseIn(pin, state);

	  SCOPE_CLOSE(INT32(us));
	}
	*/

	IMPLEMENT(delay) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, ms);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int ms = GET_ARGUMENT_AS_INT32(0);

	  ::delay(ms);

	  SCOPE_CLOSE(UNDEFINED());
	}

	IMPLEMENT(delayMicroseconds) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, us);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int us = GET_ARGUMENT_AS_INT32(0);

	  ::delayMicroseconds(us);

	  SCOPE_CLOSE(UNDEFINED());
	}

	IMPLEMENT(millis) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  unsigned int ms = ::millis();

	  SCOPE_CLOSE(UINT32(ms));
	}

	IMPLEMENT(micros) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  unsigned int us = ::micros();

	  SCOPE_CLOSE(UINT32(us));
	}

	// === Raspberry Pi specific ===

	// Func : int piBoardRev(void)
	// Description : This returns the board revision of the Raspberry Pi. It will be either 1 or 2.
	// Some of the BCM_GPIO pins changed number and function when moving from board revision 1 to 2,
	// so if you are using BCM_GPIO pin numbers, then you need to be aware of the differences.

	IMPLEMENT(piBoardRev) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  int res = ::piBoardRev();

	  SCOPE_CLOSE(INT32(res));
	}

	IMPLEMENT(piBoardId) {
	  SCOPE_OPEN();

	  CHECK_ARGUMENTS_LENGTH_EQUAL(0);

	  // libWiringPi 2.20 changes:
	  // maker is now a int indexing makerNames string tables
	  // a fifth arguments was added named overvolted
	  int model, rev, mem, marker, overvolted;

	  ::piBoardId(&model, &rev, &mem, &marker, &overvolted);

	  #if NODE_VERSION_AT_LEAST(0, 11, 0)
	    Local<Object> obj = Object::New(isolate);
	    obj->Set(String::NewFromUtf8(isolate, "model", String::kInternalizedString), INT32(model));
	    obj->Set(String::NewFromUtf8(isolate, "rev", String::kInternalizedString), INT32(rev));
	    obj->Set(String::NewFromUtf8(isolate, "mem", String::kInternalizedString), INT32(mem));
	    obj->Set(String::NewFromUtf8(isolate, "marker", String::kInternalizedString), INT32(marker));
	    obj->Set(String::NewFromUtf8(isolate, "overvolted", String::kInternalizedString), INT32(overvolted));
	  #else
	    Local<Object> obj = Object::New();
	    obj->Set(String::NewSymbol("model"), INT32(model));
	    obj->Set(String::NewSymbol("rev"), INT32(rev));
	    obj->Set(String::NewSymbol("mem"), INT32(mem));
	    obj->Set(String::NewSymbol("marker"), INT32(marker));
	    obj->Set(String::NewSymbol("overvolted"), INT32(overvolted));
	  #endif

	  SCOPE_CLOSE(obj);
	}

	// Func : int wpiPinToGpio(int wpiPin)
	// Description : This returns the BCM_GPIO pin number of the supplied wiringPi pin.
	// It takes the board revision into account.

	IMPLEMENT(wpiPinToGpio) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int res = ::wpiPinToGpio(pin);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int physPinToGpio (int physPin)
	// Description : This returns the BCM_GPIO pin number of the suppled physical pin on the P1 connector.

	IMPLEMENT(physPinToGpio) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int res = ::physPinToGpio(pin);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void setPadDrive(int group, int value)
	// Description : This sets the "strength" of the pad drivers for a particular group of pins.
	// There are 3 groups of pins and the drive strength is from 0 to 7. Do not use the unless you
	// know what you are doing.

	IMPLEMENT(setPadDrive) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, group);
	  SET_ARGUMENT_NAME(1, value);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int group = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);

	  ::setPadDrive(group, value);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : int getAlt(int pin)
	// Description : Returns the ALT bits for a given port. Only really of-use
	// for the gpio readall command (I think).

	IMPLEMENT(getAlt) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int res = ::getAlt(pin);

	  SCOPE_CLOSE(INT32(res));
	}

	IMPLEMENT(pwmToneWrite) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, frequency);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int frequency = GET_ARGUMENT_AS_INT32(1);

	  ::pwmToneWrite(pin, frequency);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void digitalWriteByte(int value)
	// Description : This writes the 8-bit byte supplied to the first 8 GPIO pins.
	// It’s the fastest way to set all 8 bits at once to a particular value, although it still takes
	// two write operations to the Pi’s GPIO hardware.

	IMPLEMENT(digitalWriteByte) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, byte);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int byte = GET_ARGUMENT_AS_INT32(0);
	  ::digitalWriteByte(byte);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pwmSetMode(int mode)
	// Description : The PWM generator can run in 2 modes – “balanced” and “mark:space”.
	// The mark:space mode is traditional, however the default mode in the Pi is “balanced”.
	// You can switch modes by supplying the parameter: PWM_MODE_BAL or PWM_MODE_MS.

	IMPLEMENT(pwmSetMode) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, mode);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int mode = GET_ARGUMENT_AS_INT32(0);

	  CHECK_ARGUMENT_IN_INTS(0, mode, (PWM_MODE_BAL, PWM_MODE_MS));

	  ::pwmSetMode(mode);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pwmSetRange(unsigned int range)
	// Description : This sets the range register in the PWM generator. The default is 1024.
	// Note: The PWM control functions can not be used when in Sys mode. To understand more about
	// the PWM system, you’ll need to read the Broadcom ARM peripherals manual.

	IMPLEMENT(pwmSetRange) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, range);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_UINT32(0);

	  unsigned int range = GET_ARGUMENT_AS_UINT32(0);
	  ::pwmSetRange(range);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void pwmSetClock(int divisor)
	// Description : This sets the divisor for the PWM clock.
	// Note: The PWM control functions can not be used when in Sys mode. To understand more about
	// the PWM system, you’ll need to read the Broadcom ARM peripherals manual.

	IMPLEMENT(pwmSetClock) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, divisor);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int divisor = GET_ARGUMENT_AS_INT32(0);
	  ::pwmSetClock(divisor);

	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void gpioClockSet(int pin, int freq)
	// Description : Set the frequency on a GPIO clock pin

	IMPLEMENT(gpioClockSet) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, frequency);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int frequency = GET_ARGUMENT_AS_INT32(1);

	  ::gpioClockSet(pin, frequency);

	  SCOPE_CLOSE(UNDEFINED());
	}











	IMPLEMENT(drcSetupSerial) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, numPins);
	  SET_ARGUMENT_NAME(2, device);
	  SET_ARGUMENT_NAME(3, baudrate);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(4);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_STRING(2);
	  CHECK_ARGUMENT_TYPE_INT32(3);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int numPins = GET_ARGUMENT_AS_INT32(1);
	  #if NODE_VERSION_AT_LEAST(0, 11, 0)
	    String::Utf8Value device(GET_ARGUMENT_AS_STRING(2));
	  #else
	    String::AsciiValue device(GET_ARGUMENT_AS_STRING(2));
	  #endif
	  int baudrate = GET_ARGUMENT_AS_INT32(3);
	  
	  int res = ::drcSetupSerial(pinBase, numPins, *device, baudrate);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int max5233Setup(int pinBase, int spiChannel)

	IMPLEMENT(max5322Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  int res = ::max5322Setup(pinBase, spiChannel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	IMPLEMENT(max31855Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  int res = ::max31855Setup(pinBase, spiChannel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func int mcp23s08Setup(const int pinBase, const int spiChannel, const int devId)

	IMPLEMENT(mcp23s08Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  SET_ARGUMENT_NAME(2, devId);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  int devId = GET_ARGUMENT_AS_INT32(2);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  //MCP23S08 3bits addressing
	  CHECK_ARGUMENT_IN_RANGE(2, devId, 0, 7);
	  
	  int res = ::mcp23s08Setup(pinBase, spiChannel, devId);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp23s17Setup(int pinBase, int spiPort, int devId)
	// Description : Initialise libWiringPi to be used with MCP23S17
	// pinBase is any number above 64 that doesn’t clash with any other wiringPi expansion module, 
	// spiPort is 0 or 1 for one of the two SPI ports on the Pi and devId is the ID of that MCP23s17 on the SPI port.

	IMPLEMENT(mcp23s17Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  SET_ARGUMENT_NAME(2, devId);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  int devId = GET_ARGUMENT_AS_INT32(2);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  //MCP23S17 3bits addressing
	  CHECK_ARGUMENT_IN_RANGE(2, devId, 0, 7);
	  
	  int res = ::mcp23s17Setup(pinBase, spiChannel, devId);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp3002Setup(int pinBase, int spiChannel)

	IMPLEMENT(mcp3002Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  int res = ::mcp3002Setup(pinBase, spiChannel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp3004Setup(int pinBase, int spiChannel)

	IMPLEMENT(mcp3004Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  int res = ::mcp3004Setup(pinBase, spiChannel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp3422Setup(int pinBase, int i2cAddress, int sampleRate, int gain)

	IMPLEMENT(mcp3422Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  SET_ARGUMENT_NAME(2, sampleRate);
	  SET_ARGUMENT_NAME(3, gain);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(4);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  CHECK_ARGUMENT_TYPE_INT32(3);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  int sampleRate = GET_ARGUMENT_AS_INT32(2);
	  int gain = GET_ARGUMENT_AS_INT32(3);
	  
	  CHECK_ARGUMENT_IN_INTS(2, sampleRate, (MCP3422_SR_3_75, MCP3422_SR_15, MCP3422_SR_60, MCP3422_SR_240));
	  CHECK_ARGUMENT_IN_INTS(3, gain, (MCP3422_GAIN_1, MCP3422_GAIN_2, MCP3422_GAIN_4, MCP3422_GAIN_8));
	  
	  int res = ::mcp3422Setup(pinBase, i2cAddress, sampleRate, gain);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp4802Setup(int pinBase, int spiChannel)

	IMPLEMENT(mcp4802Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, spiChannel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int spiChannel = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(1, spiChannel, (0, 1));
	  
	  int res = ::mcp4802Setup(pinBase, spiChannel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp23008Setup(int pinBase, int i2cAddress)

	IMPLEMENT(mcp23008Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::mcp23008Setup(pinBase, i2cAddress);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp23016Setup(int pinBase, int i2cAddress)

	IMPLEMENT(mcp23016Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::mcp23016Setup(pinBase, i2cAddress);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int mcp23017Setup(int pinBase, int i2cAddress)

	IMPLEMENT(mcp23017Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::mcp23017Setup(pinBase, i2cAddress);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int pcf8574Setup(const int pinBase, const int i2cAddress)

	IMPLEMENT(pcf8574Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::pcf8574Setup(pinBase, i2cAddress);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int pcf8591Setup(const int pinBase, const int i2cAddress)

	IMPLEMENT(pcf8591Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, i2cAddress);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int i2cAddress = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::pcf8591Setup(pinBase, i2cAddress);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int sn3128Setup(int pinBase)

	IMPLEMENT(sn3218Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  
	  int res = ::sn3218Setup(pinBase);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int sr595Setup(const int pinBase, const int numPins, const int dataPin, const int clockPin, const int latchPin)

	IMPLEMENT(sr595Setup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pinBase);
	  SET_ARGUMENT_NAME(1, numPins);
	  SET_ARGUMENT_NAME(2, dataPin);
	  SET_ARGUMENT_NAME(3, clockPin);
	  SET_ARGUMENT_NAME(4, latchPin);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(5);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  CHECK_ARGUMENT_TYPE_INT32(3);
	  CHECK_ARGUMENT_TYPE_INT32(4);
	  
	  int pinBase = GET_ARGUMENT_AS_INT32(0);
	  int numPins = GET_ARGUMENT_AS_INT32(1);
	  int dataPin = GET_ARGUMENT_AS_INT32(2);
	  int clockPin = GET_ARGUMENT_AS_INT32(3);
	  int latchPin = GET_ARGUMENT_AS_INT32(4);
	  
	  int res = ::sr595Setup(pinBase, numPins, dataPin, clockPin, latchPin);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int softPwmCreate(int pin, int value, int range)
	// Description : This creates a software controlled PWM pin. 
	// You can use any GPIO pin and the pin numbering will be that of the wiringPiSetup() function you used. 
	// Use 100 for the pwmRange, then the value can be anything from 0 (off) to 100 (fully on) for the given pin.
	// The return value is 0 for success. Anything else and you should check the global errno variable to see what went wrong.
	// NOTE : You must initialise wiringPi with one of wiringPiSetup(),  wiringPiSetupGpio() or wiringPiSetupPhys() functions. 
	// wiringPiSetupSys() is not fast enough, so you must run your programs with sudo.
	// NOTE2 : Each “cycle” of PWM output takes 10mS with the default range value of 100, 
	// so trying to change the PWM value more than 100 times a second will be futile.
	// NOTE3 : Each pin activated in softPWM mode uses approximately 0.5% of the CPU.
	// NOTE4 : There is currently no way to disable softPWM on a pin while the program in running.
	// NOTE5 : You need to keep your program running to maintain the PWM output!

	IMPLEMENT(softPwmCreate) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, value);
	  SET_ARGUMENT_NAME(2, range);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  
	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);
	  int range = GET_ARGUMENT_AS_INT32(2);
	  
	  int res = ::softPwmCreate(pin, value, range);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func void softPwmWrite(int pin, int value)
	// Description : This updates the PWM value on the given pin. 
	// The value is checked to be in-range and pins that haven’t previously been initialised via softPwmCreate will be silently ignored.

	IMPLEMENT(softPwmWrite) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, value);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);
	  
	  ::softPwmWrite(pin, value);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	IMPLEMENT(softPwmStop) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pin);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int pin = GET_ARGUMENT_AS_INT32(0);
	  
	  ::softPwmStop(pin);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void softServoWrite(int pin, int value)
	// Description : Write a Servo value to the given pin

	IMPLEMENT(softServoWrite) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, pin);
	  SET_ARGUMENT_NAME(1, value);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int pin = GET_ARGUMENT_AS_INT32(0);
	  int value = GET_ARGUMENT_AS_INT32(1);
	  
	  ::softServoWrite(pin, value);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : int softServoSetup(int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7)
	// Description : Setup the software servo system

	IMPLEMENT(softServoSetup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, p0);
	  SET_ARGUMENT_NAME(1, p1);
	  SET_ARGUMENT_NAME(2, p2);
	  SET_ARGUMENT_NAME(3, p3);
	  SET_ARGUMENT_NAME(4, p4);
	  SET_ARGUMENT_NAME(5, p5);
	  SET_ARGUMENT_NAME(6, p6);
	  SET_ARGUMENT_NAME(7, p7);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(8);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  CHECK_ARGUMENT_TYPE_INT32(3);
	  CHECK_ARGUMENT_TYPE_INT32(4);
	  CHECK_ARGUMENT_TYPE_INT32(5);
	  CHECK_ARGUMENT_TYPE_INT32(6);
	  CHECK_ARGUMENT_TYPE_INT32(7);
	  
	  int p0 = GET_ARGUMENT_AS_INT32(0);
	  int p1 = GET_ARGUMENT_AS_INT32(1);
	  int p2 = GET_ARGUMENT_AS_INT32(2);
	  int p3 = GET_ARGUMENT_AS_INT32(3);
	  int p4 = GET_ARGUMENT_AS_INT32(4);
	  int p5 = GET_ARGUMENT_AS_INT32(5);
	  int p6 = GET_ARGUMENT_AS_INT32(6);
	  int p7 = GET_ARGUMENT_AS_INT32(7);
	  
	  int res = ::softServoSetup(p0, p1, p2, p3, p4, p5, p6, p7);
	  
	  SCOPE_CLOSE(INT32(res));
	}








	// Func : int wiringPiI2CRead (int fd);
	// Simple device read. Some devices present data when you read them without having to do any register transactions.

	IMPLEMENT(wiringPiI2CRead) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  
	  int res = ::wiringPiI2CRead(fd);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiI2CRead (int fd, int reg);
	// read an 8-bits value from the device register indicated.

	IMPLEMENT(wiringPiI2CReadReg8) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, reg);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  int reg = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::wiringPiI2CReadReg8(fd, reg);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiI2CRead (int fd, int reg)
	// read a 16-bits value from the device register indicated.

	IMPLEMENT(wiringPiI2CReadReg16) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, reg);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);

	  int fd = GET_ARGUMENT_AS_INT32(0);
	  int reg = GET_ARGUMENT_AS_INT32(1);

	  int res = ::wiringPiI2CReadReg16(fd, reg);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiI2CWrite (int fd, int data)
	// Simple device write. Some devices accept data this way without needing to access any internal registers.

	IMPLEMENT(wiringPiI2CWrite) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, data);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  int data = GET_ARGUMENT_AS_INT32(1);
	  data = data & 0xFF;
	  
	  int res = ::wiringPiI2CWrite(fd, data);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiI2CWriteReg8 (int fd, int reg, int data)
	// write an 8-bit data value into the device register indicated.

	IMPLEMENT(wiringPiI2CWriteReg8) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, reg);
	  SET_ARGUMENT_NAME(2, data);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);

	  int fd = GET_ARGUMENT_AS_INT32(0);
	  int reg = GET_ARGUMENT_AS_INT32(1);
	  int data = GET_ARGUMENT_AS_INT32(2);
	  data = data & 0xFF;

	  int res = ::wiringPiI2CWriteReg8(fd, reg, data);

	  SCOPE_CLOSE(INT32(res));
	}

	IMPLEMENT(wiringPiI2CWriteReg16) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, reg);
	  SET_ARGUMENT_NAME(2, data);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);

	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);

	  int fd = GET_ARGUMENT_AS_INT32(0);
	  int reg = GET_ARGUMENT_AS_INT32(1);
	  int data = GET_ARGUMENT_AS_INT32(2);
	  data = data & 0xFFFF;

	  int res = ::wiringPiI2CWriteReg16(fd, reg, data);

	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiI2CSetupInterface (const char *device, int devId)

	IMPLEMENT(wiringPiI2CSetupInterface) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, device);
	  SET_ARGUMENT_NAME(1, devId);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_STRING(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  #if NODE_VERSION_AT_LEAST(0, 11, 0)
	    String::Utf8Value device(GET_ARGUMENT_AS_STRING(0));
	  #else
	    String::AsciiValue device(GET_ARGUMENT_AS_STRING(0));
	  #endif
	  int devId = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::wiringPiI2CSetupInterface(*device, devId);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wirintPiI2CSetup (int devId)

	IMPLEMENT(wiringPiI2CSetup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, devId);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int devId = GET_ARGUMENT_AS_INT32(0);
	  
	  int res = ::wiringPiI2CSetup(devId);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void wiringPiI2CClose(const int fd)
	// Description : This closes opened I2C file descriptor
	// fd is file descriptor returned either from wiringPiI2CSetup or wiringPiI2CSetupInterface

	IMPLEMENT(wiringPiI2CClose) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, fd);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int fd = GET_ARGUMENT_AS_INT32(0);

	  ::close(fd);

	  SCOPE_CLOSE(UNDEFINED());
	}
















	// Func : int wiringPiSPIGetFd(int channel)

	IMPLEMENT(wiringPiSPIGetFd) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, channel);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int channel = GET_ARGUMENT_AS_INT32(0);
	  
	  CHECK_ARGUMENT_IN_INTS(0, channel, (0, 1));
	  
	  int res = ::wiringPiSPIGetFd(channel);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : wiringPiSPIDataRW(int channel, unsigned char* data, int len)

	IMPLEMENT(wiringPiSPIDataRW) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, channel);
	  SET_ARGUMENT_NAME(1, data);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_NODE_BUFFER(1);
	  
	  int channel = GET_ARGUMENT_AS_INT32(0);
	  char* data = node::Buffer::Data(args[1]->ToObject());
	  int length = node::Buffer::Length(args[1]->ToObject());
	  
	  CHECK_ARGUMENT_IN_INTS(0, channel, (0, 1));
	  
	  int res = ::wiringPiSPIDataRW(channel, reinterpret_cast<unsigned char*>(data), length);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int wiringPiSPISetup(int channel, int speed)

	IMPLEMENT(wiringPiSPISetup) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, channel);
	  SET_ARGUMENT_NAME(1, speed);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  int channel = GET_ARGUMENT_AS_INT32(0);
	  int speed = GET_ARGUMENT_AS_INT32(1);
	  
	  CHECK_ARGUMENT_IN_INTS(0, channel, (0, 1));
	  
	  int res = ::wiringPiSPISetup(channel, speed);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	IMPLEMENT(wiringPiSPISetupMode) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, channel);
	  SET_ARGUMENT_NAME(1, speed);
	  SET_ARGUMENT_NAME(2, mode);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  CHECK_ARGUMENT_TYPE_INT32(2);
	  
	  int channel = GET_ARGUMENT_AS_INT32(0);
	  int speed = GET_ARGUMENT_AS_INT32(1);
	  int mode = GET_ARGUMENT_AS_INT32(2);
	  
	  CHECK_ARGUMENT_IN_INTS(0, channel, (0, 1));
	  CHECK_ARGUMENT_IN_INTS(2, mode, (0, 1, 2, 3));
	  
	  int res = ::wiringPiSPISetupMode(channel, speed, mode);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void wiringPiSPIClose(const int fd)
	// Description : This closes opened SPI file descriptor
	// fd is file descriptor returned either from wiringPiSPISetup or wiringPiSPISetupMode

	IMPLEMENT(wiringPiSPIClose) {
	  SCOPE_OPEN();

	  SET_ARGUMENT_NAME(0, fd);

	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);

	  CHECK_ARGUMENT_TYPE_INT32(0);

	  int fd = GET_ARGUMENT_AS_INT32(0);

	  ::close(fd);

	  SCOPE_CLOSE(UNDEFINED());
	}


	// Func : int serialOpen(const char* device, const int baud)

	IMPLEMENT(serialOpen) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, device);
	  SET_ARGUMENT_NAME(1, baudrate);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_STRING(0);
	  CHECK_ARGUMENT_TYPE_INT32(1);
	  
	  #if NODE_VERSION_AT_LEAST(0, 11, 0)
	    String::Utf8Value device(GET_ARGUMENT_AS_STRING(0));
	  #else
	    String::AsciiValue device(GET_ARGUMENT_AS_STRING(0));
	  #endif
	  int baudrate = GET_ARGUMENT_AS_INT32(1);
	  
	  int res = ::serialOpen(*device, baudrate);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : void serialClose(const int fd)

	IMPLEMENT(serialClose) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  
	  ::serialClose(fd);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void serialFlush(const int fd);

	IMPLEMENT(serialFlush) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  
	  ::serialFlush(fd);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void serialPutchar(const int fd, const unsigned char c)

	IMPLEMENT(serialPutchar) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, character);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_UINT32(1);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  unsigned char character = GET_ARGUMENT_AS_UINT32(1);
	  
	  ::serialPutchar(fd, character);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void serialPuts(const int fd, const char* s)

	IMPLEMENT(serialPuts) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  SET_ARGUMENT_NAME(1, string);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(2);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  CHECK_ARGUMENT_TYPE_STRING(1);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  #if NODE_VERSION_AT_LEAST(0, 11, 0)
	    String::Utf8Value string(GET_ARGUMENT_AS_STRING(1));
	  #else
	    String::AsciiValue string(GET_ARGUMENT_AS_STRING(1));
	  #endif
	  
	  ::serialPuts(fd, *string);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	// Func : void serialPrintf(const int fd, const char* message, ...)

	IMPLEMENT(serialPrintf) {
	  // Make serialPrintf a alias to serialPuts
	  return serialPuts(args);
	}

	// Func : int serialDataAvail(const int fd)

	IMPLEMENT(serialDataAvail) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  
	  int res = ::serialDataAvail(fd);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : int serialGetchar(const int fd)
	// NOTE TO MYSELF : I don't understand why serialPutchar takes a unsigned char and on the other side
	// serialGetchar returns a int ... serialGetchar should returns a unsigned char too.

	IMPLEMENT(serialGetchar) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, fd);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(1);
	  
	  CHECK_ARGUMENT_TYPE_INT32(0);
	  
	  int fd = GET_ARGUMENT_AS_INT32(0);
	  
	  int res = ::serialGetchar(fd);
	  
	  SCOPE_CLOSE(INT32(res));
	}

	// Func : uint8_t shiftIn(uint8_t dPin, uint8_t cPin, uint8_t order)
	// Description : This shifts an 8-bit data value in with the data appearing on the dPin and the clock being sent out on the cPin.
	// Order is either LSBFIRST or MSBFIRST.
	// The data is sampled after the cPin goes high.
	// (So cPin high, sample data, cPin low, repeat for 8 bits) The 8-bit value is returned by the function.

	IMPLEMENT(shiftIn) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, dPin);
	  SET_ARGUMENT_NAME(1, cPin);
	  SET_ARGUMENT_NAME(2, order);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(3);
	  
	  CHECK_ARGUMENT_TYPE_UINT32(0);
	  CHECK_ARGUMENT_TYPE_UINT32(1);
	  CHECK_ARGUMENT_TYPE_UINT32(2);
	  
	  uint8_t dPin = GET_ARGUMENT_AS_UINT32(0);
	  uint8_t cPin = GET_ARGUMENT_AS_UINT32(1);
	  uint8_t order = GET_ARGUMENT_AS_UINT32(2);
	  
	  CHECK_ARGUMENT_IN_INTS(2, order, (LSBFIRST, MSBFIRST));
	  
	  uint8_t res = ::shiftIn(dPin, cPin, order);
	  
	  SCOPE_CLOSE(UINT32(res));
	}

	// Func : void shiftOut(uint8_t dPin, uint8_t cPin, uint8_t order, uint8_t val) ;
	// Description : The shifts an 8-bit data value val out with the data being sent out on dPin and the clock being sent out on the cPin.
	// order is as above. 
	// Data is clocked out on the rising or falling edge – ie. dPin is set, then cPin is taken high then low – repeated for the 8 bits.

	IMPLEMENT(shiftOut) {
	  SCOPE_OPEN();
	  
	  SET_ARGUMENT_NAME(0, dPin);
	  SET_ARGUMENT_NAME(1, cPin);
	  SET_ARGUMENT_NAME(2, order);
	  SET_ARGUMENT_NAME(3, value);
	  
	  CHECK_ARGUMENTS_LENGTH_EQUAL(4);
	  
	  CHECK_ARGUMENT_TYPE_UINT32(0);
	  CHECK_ARGUMENT_TYPE_UINT32(1);
	  CHECK_ARGUMENT_TYPE_UINT32(2);
	  CHECK_ARGUMENT_TYPE_UINT32(3);
	  
	  uint8_t dPin = GET_ARGUMENT_AS_UINT32(0);
	  uint8_t cPin = GET_ARGUMENT_AS_UINT32(1);
	  uint8_t order = GET_ARGUMENT_AS_UINT32(2);
	  uint8_t value = GET_ARGUMENT_AS_UINT32(3);
	  
	  CHECK_ARGUMENT_IN_INTS(2, order, (LSBFIRST, MSBFIRST));
	  
	  ::shiftOut(dPin, cPin, order, value);
	  
	  SCOPE_CLOSE(UNDEFINED());
	}

	void init(Local<Object> exports) { // 중요 3. Method 정의
		
		//
		EXPORT_FUNCTION(world);
		
		// Setup
	  //EXPORT_FUNCTION(setup);
	  EXPORT_FUNCTION(wiringPiSetup);
	  EXPORT_FUNCTION(wiringPiSetupGpio);
	  EXPORT_FUNCTION(wiringPiSetupSys);
	  EXPORT_FUNCTION(wiringPiSetupPhys);

	  // Core functions
	  EXPORT_FUNCTION(pinModeAlt);
	  EXPORT_FUNCTION(pinMode);
	  EXPORT_FUNCTION(pullUpDnControl);
	  EXPORT_FUNCTION(digitalRead);
	  EXPORT_FUNCTION(digitalWrite);
	  EXPORT_FUNCTION(pwmWrite);
	  EXPORT_FUNCTION(analogRead);
	  EXPORT_FUNCTION(analogWrite);
	  //EXPORT_FUNCTION(pulseIn);

	  EXPORT_FUNCTION(delay);
	  EXPORT_FUNCTION(delayMicroseconds);
	  EXPORT_FUNCTION(millis);
	  EXPORT_FUNCTION(micros);

	  // On-Board Rasberry Pi hardware specific stuff
	  EXPORT_FUNCTION(piBoardRev);
	  EXPORT_FUNCTION(piBoardId);
	  EXPORT_FUNCTION(wpiPinToGpio);
	  EXPORT_FUNCTION(physPinToGpio);
	  EXPORT_FUNCTION(setPadDrive);
	  EXPORT_FUNCTION(getAlt);
	  EXPORT_FUNCTION(pwmToneWrite);
	  EXPORT_FUNCTION(digitalWriteByte);
	  EXPORT_FUNCTION(pwmSetMode);
	  EXPORT_FUNCTION(pwmSetRange);
	  EXPORT_FUNCTION(pwmSetClock);
	  EXPORT_FUNCTION(gpioClockSet);
	  
	  // Extensions
	  EXPORT_FUNCTION(drcSetupSerial);
	  EXPORT_FUNCTION(max5322Setup);
	  EXPORT_FUNCTION(max31855Setup);
	  EXPORT_FUNCTION(mcp23s08Setup);
	  EXPORT_FUNCTION(mcp23s17Setup);
	  EXPORT_FUNCTION(mcp3002Setup);
	  EXPORT_FUNCTION(mcp3004Setup);
	  EXPORT_FUNCTION(mcp3422Setup);
	  EXPORT_FUNCTION(mcp4802Setup);
	  EXPORT_FUNCTION(mcp23008Setup);
	  EXPORT_FUNCTION(mcp23016Setup);
	  EXPORT_FUNCTION(mcp23017Setup);
	  EXPORT_FUNCTION(pcf8574Setup);
	  EXPORT_FUNCTION(pcf8591Setup);
	  EXPORT_FUNCTION(sn3218Setup);
	  EXPORT_FUNCTION(sr595Setup);
	//  EXPORT_FUNCTION(pca9685Setup);
	  
	  // Soft PWM
	  EXPORT_FUNCTION(softPwmCreate);
	  EXPORT_FUNCTION(softPwmWrite);
	  EXPORT_FUNCTION(softPwmStop);
	  
	  // Soft Servo
	  EXPORT_FUNCTION(softServoWrite);
	  EXPORT_FUNCTION(softServoSetup);
	  
	  // Soft Tone
	//  EXPORT_FUNCTION(softToneCreate);
	//  EXPORT_FUNCTION(softToneWrite);
	//  EXPORT_FUNCTION(softToneStop);
	  
	  // WiringPI I2C
	  EXPORT_FUNCTION(wiringPiI2CRead);
	  EXPORT_FUNCTION(wiringPiI2CReadReg8);
	  EXPORT_FUNCTION(wiringPiI2CReadReg16);
	  EXPORT_FUNCTION(wiringPiI2CWrite);
	  EXPORT_FUNCTION(wiringPiI2CWriteReg8);
	  EXPORT_FUNCTION(wiringPiI2CWriteReg16);
	  EXPORT_FUNCTION(wiringPiI2CSetupInterface);
	  EXPORT_FUNCTION(wiringPiI2CSetup);
	  EXPORT_FUNCTION(wiringPiI2CClose);
	  
	  // WiringPI SPI
	  EXPORT_FUNCTION(wiringPiSPIGetFd);
	  EXPORT_FUNCTION(wiringPiSPIDataRW);
	  EXPORT_FUNCTION(wiringPiSPISetup);
	  EXPORT_FUNCTION(wiringPiSPISetupMode);
	  EXPORT_FUNCTION(wiringPiSPIClose);
	  
	  // WiringPi Serial
	  EXPORT_FUNCTION(serialOpen);
	  EXPORT_FUNCTION(serialClose);
	  EXPORT_FUNCTION(serialFlush);
	  EXPORT_FUNCTION(serialPutchar);
	  EXPORT_FUNCTION(serialPuts);
	  EXPORT_FUNCTION(serialPrintf);
	  EXPORT_FUNCTION(serialDataAvail);
	  EXPORT_FUNCTION(serialGetchar);
	  
	  // WiringPi Shift
	  EXPORT_FUNCTION(shiftIn);
	  EXPORT_FUNCTION(shiftOut);
	  
	  /*	  
	  Isolate* isolate = Isolate::GetCurrent();
	  
	  // WPI_MODEs
	  EXPORT_CONSTANT_INT(WPI_MODE_PINS);
	  EXPORT_CONSTANT_INT(WPI_MODE_PHYS);
	  EXPORT_CONSTANT_INT(WPI_MODE_GPIO);
	  EXPORT_CONSTANT_INT(WPI_MODE_GPIO_SYS);
	  EXPORT_CONSTANT_INT(WPI_MODE_PIFACE);
	  EXPORT_CONSTANT_INT(WPI_MODE_UNINITIALISED);
	  

	  // pinMode
	  EXPORT_CONSTANT_INT(INPUT);
	  EXPORT_CONSTANT_INT(OUTPUT);
	  EXPORT_CONSTANT_INT(PWM_OUTPUT);
	  EXPORT_CONSTANT_INT(GPIO_CLOCK);
	  EXPORT_CONSTANT_INT(SOFT_PWM_OUTPUT);
	  EXPORT_CONSTANT_INT(SOFT_TONE_OUTPUT);
	  
	  // pullUpDnControl
	  EXPORT_CONSTANT_INT(PUD_OFF);
	  EXPORT_CONSTANT_INT(PUD_DOWN);
	  EXPORT_CONSTANT_INT(PUD_UP);

	  // digitalRead/Write
	  EXPORT_CONSTANT_INT(HIGH);
	  EXPORT_CONSTANT_INT(LOW);

	  // pwmSetMode
	  EXPORT_CONSTANT_INT(PWM_MODE_BAL);
	  EXPORT_CONSTANT_INT(PWM_MODE_MS);

	  // piBoardId
	  //EXPORT_CONSTANT_INT(PI_MODEL_UNKNOWN);
	  EXPORT_CONSTANT_INT(PI_MODEL_A);
	  EXPORT_CONSTANT_INT(PI_MODEL_B);
	  EXPORT_CONSTANT_INT(PI_MODEL_BP);
	  EXPORT_CONSTANT_INT(PI_MODEL_CM);
	  EXPORT_CONSTANT_INT(PI_MODEL_AP);
	  EXPORT_CONSTANT_INT(PI_MODEL_2);

	  EXPORT_CONSTANT_INT(PI_VERSION_UNKNOWN);
	  EXPORT_CONSTANT_INT(PI_VERSION_1);
	  EXPORT_CONSTANT_INT(PI_VERSION_1_1);
	  EXPORT_CONSTANT_INT(PI_VERSION_1_2);
	  EXPORT_CONSTANT_INT(PI_VERSION_2);

	  EXPORT_CONSTANT_INT(PI_MAKER_UNKNOWN);
	  EXPORT_CONSTANT_INT(PI_MAKER_EGOMAN);
	  EXPORT_CONSTANT_INT(PI_MAKER_SONY);
	  EXPORT_CONSTANT_INT(PI_MAKER_QISDA);
	  EXPORT_CONSTANT_INT(PI_MAKER_MBEST);

	  EXPORT_CONSTANT_STRING_ARRAY(PI_MODEL_NAMES, piModelNames, 7);
	  EXPORT_CONSTANT_STRING_ARRAY(PI_REVISION_NAMES, piRevisionNames, 5);
	  EXPORT_CONSTANT_STRING_ARRAY(PI_MAKER_NAMES, piMakerNames, 5);

	  // pinModeAlt
	  EXPORT_CONSTANT_INT(FSEL_INPT);
	  EXPORT_CONSTANT_INT(FSEL_OUTP);
	  EXPORT_CONSTANT_INT(FSEL_ALT0);
	  EXPORT_CONSTANT_INT(FSEL_ALT1);
	  EXPORT_CONSTANT_INT(FSEL_ALT2);
	  EXPORT_CONSTANT_INT(FSEL_ALT3);
	  EXPORT_CONSTANT_INT(FSEL_ALT4);
	  EXPORT_CONSTANT_INT(FSEL_ALT5);
	  */
	}

	NODE_MODULE(wiringPi, init)
}