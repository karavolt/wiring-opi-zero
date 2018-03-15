// Example of using PWM
var wpi = require('wiring-bpi-zero');
var async = require('async');

wpi.setup();

var pin = 7;
wpi.pinMode(pin, wpi.PWM_OUTPUT);
//wpi.pwmSetRange(1024);

async.series([
  function (cb) {
    wpi.pwmWrite(pin, 100);
    cb();
  },
  function (cb) {
    setTimeout(cb, 3000);
  },
  function (cb) {
    wpi.pwmWrite(pin, 1023);
    cb();
  },
  function (cb) {
    setTimeout(cb, 3000);
  },
  function (cb) {
    wpi.pwmWrite(pin, 0);
    cb();
  }
], function (err) {
});
