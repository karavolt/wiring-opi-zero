var wpi = require('wiring-bpi-zero');

//var str = wpi.world()
//console.log("str : " + str)

wpi.setup();

var pin = 25;

wpi.pinMode(pin, wpi.OUTPUT);

var value = 1;

setInterval(function() {
  wpi.digitalWrite(pin, value);
  value = +!value;
}, 500);

