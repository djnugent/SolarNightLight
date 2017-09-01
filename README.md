# Solar Night Light
![Light at night](https://raw.githubusercontent.com/djnugent/SolarNightLight/master/pics/IMG_20161215_232419544.jpg)
I made a solar night for my girlfriend's Christmas present. She's an advocate for green energy and her first band(The Roomies) just parted ways so I decided to make her a night light that encompassed both of these. For this project I created a 3D printed enclosure covered in veneer and custom solar circuit based on a attiny85.

## Circuit
![Circuit](https://raw.githubusercontent.com/djnugent/SolarNightLight/master/pics/IMG_20161206_144125350.jpg)

The first step was to test design and prototype the circuit to make sure my idea was possible. I wanted it to be solar power(obviously) but also rechargeable by usb if there wasn't enough sun.  I choose a 6v 70mA solar panel and 3 AA 2500mAh rechargable batteries to power the system. The system draws 20mA when lit and &lt;.10mA when idle. So it could roughly last 10 days on a single charge with no sun. I used an attiny85(adafruit 3v trinket) to read solar levels and control the led panel brightness with pwm. The led panel was composed of 8 12mmx40mm led backlights that were masked by a sticker from carstickers.com. I was pretty proud of the circuit because it is simple but is actually does a lot. It allows for solar or usb charging and uses the solar panel as a light sensor. I designed it myself and it works great!

## Enclosure
![Enclosure](https://raw.githubusercontent.com/djnugent/SolarNightLight/master/pics/Nightlight.png)

I CADed the enclosure up in sketchup(because I haven't taken the time to learn a parametric CAD suite yet). It is composed of 3 parts and was printed on my printrbot simple using Simplify3D.  I had to print it twice because the first version was about a 1mm to small in every dimension; Damn tolerancing. There was access for the usb port. That back has access for battery replacement(if it every needed). The front is open for led panel to shine through the veneer.

## Veneer
The veneer was the key to making this project look nice. I really wanted to make the night light out of wood, not plastic, but I don't have a woodshop in my apartment so that was out of the question. The veneer also allowed for the signiture look of the text shining through the wood. It give a nice warm glow at night and looks like a simple box in the window during the day.

## Replicate
All the design files, code, and photos can be found here!
