---
title: "swarm drones"
github: "https://github.com/SolderBits/swarm-drone.git"
description: "Trying to build my own swarm drones project"
created_at: "2026-10-18"
total_time: "36h 48m"
---

# November 5, 2026: the Ideations 

It all started when I watched a video on a drone light show. I was honestly amazed by how so many drones could fly together and make those cool patterns. I don't know much about drones at all so naturally i researched a little.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   I started looking into how to make a few drones and make a litting show like thiss but got to know its not as simple as it looks and took up this challenge.
First i started with the normal generic drone ppl build thru the videos with those generic goldem motors but after a quick bom i got to know its costs way too much and not complex enough for hackclub to grant me funds so after some search i found this esp mini drone which i wanted to build but idk how to do this ligthing show so i went online any cheaper alternatives andf got to known abt a cool repo by joshua bird (https://github.com/jyjblrd/Low-Cost-Mocap) where he uses ps3 camera to get the position of the esp drone which is cheap and could acheive my aim with it so i started building this project.

![](https://fabricate.hackclub-assets.com/193d8a43265de0e5347b942bfabefc9c07384d03a95e069f9f1ea970c233a617/image-png/Screenshot%202026-11-05%20at%204.27.24%E2%80%AFPM.png)

**Total time spent: 3h 30m**

# November 5, 2026: the schematics

Now ive started to look into the pcb part and took inspo from the actual repo and and the esp fly drone design and started designing it around that and it was pretty straight forward and my previous exp with kicad helped me alot in this just finding the layout was the biggest pain in my ass so i ended up asking ai for the fc pcb to make and thats why they look a little off and everything else was done by me and did the schematics part for both the led chaser with the 555 timer for the ir leds and the fc unit schematics.

![](https://fabricate.hackclub-assets.com/1953bed1a8f00fd153fbdcd37b9b9a8061ffed58bf47f55032d99f0057c6b401/image-jpeg/PHOTO-2026-11-05-15-28-36.jpg)
 

![](https://fabricate.hackclub-assets.com/196a865c72504965b2f8b4d9e88d7ccb6c476be50f12d6e6016881f470a4399e/image-png/Screenshot%202026-11-05%20at%207.02.19%E2%80%AFPM.png)
![](https://fabricate.hackclub-assets.com/bfdc0db114f1c9acb8e12fe9f09bacf490793fe999d39a971ca708b777d6b722/image-png/Screenshot%202026-11-05%20at%207.02.51%E2%80%AFPM.png)

**Total time spent: 5h 42m**

# November 5, 2026: the final pcb 

ive started routing the pcbs it took some time and alot of vias later they ended up pretty well and look staright forward to start with i just connected eveyrthing as per the schematics and add a gnd plane suggested by a yt video on kicad to reduce noise on top of it .

![](https://fabricate.hackclub-assets.com/319668071329a659bb4e99dbec6300e5a3327ae3d64753841a21af257b80180d/image-png/Screenshot%202026-11-05%20at%208.10.50%E2%80%AFPM.png)
![](https://fabricate.hackclub-assets.com/0f7dc7d229ff1eeefcabb40933744557ee8d50fcf5d0555a170d41659fdc5822/image-png/Screenshot%202026-11-05%20at%207.48.40%E2%80%AFPM.png)

**Total time spent: 4h 30m**

# November 5, 2026: the Frame

after a lot of cad tutorials and getting my fusion activated on my student mail then after alot of back and forth ive finally designed the frame it kinda looks mediocre but im sure it flies fs
![](https://fabricate.hackclub-assets.com/3fb10ab99f881f99773a19a95af7bbd14fc5473de7d79ac1e359e3a307b25870/image-png/Screenshot%202026-11-05%20at%208.31.37%E2%80%AFPM.png)

**Total time spent: 2h 42m**

# November 13, 2026: NEW CHANGES 

After a little work thru having two diff micro controllers made it really hard so brought it down to a single microcontroller for everything and hopping everything goes well and made a new design for everything.

![](https://fabricate.hackclub-assets.com/a2fbbb0653eb15c1a371fd578e10414238c6f2a39678f6547822991f60d92743/image-png/Screenshot%202026-11-13%20at%2010.27.27%E2%80%AFPM.png)
![](https://fabricate.hackclub-assets.com/3a20e7c00d2d261f378a68a275277abd4d7217f97192336c70b01fe06804763a/image-png/Screenshot%202026-11-13%20at%2010.27.54%E2%80%AFPM.png)

**Total time spent: 4h 30m**

# November 13, 2026: NEW frame

with the new pcb design ive even started to design the new frame around it nothing flashy smtg simple asf for the drone and which is light at the same time 

![](https://fabricate.hackclub-assets.com/0b9a94365df242e009fc71a55f3c12d6d1e0dc9d3552b327c66f046c68efb49a/image-png/Screenshot%202026-11-13%20at%2010.32.29%E2%80%AFPM.png)
![](https://fabricate.hackclub-assets.com/49e8a1de150a0b400f59ae0d300915389684e6574d2a89a93c816273879a96d8/image-png/Screenshot%202026-11-13%20at%2010.32.34%E2%80%AFPM.png)

**Total time spent: 3h 42m**

# November 13, 2026: the motion tracking integration

after getting the schematics done i started looking into the software side of things. since im using an esp based flight controller i had to figure out how the code would actually control the motors and handle the drone.i started looking at how the esp communicates with the different components and how the motor outputs would work. i also had to think about how the ir leds would be controlled so that the cameras could detect the drone properly.the tricky part was understanding how all these things come together. i didnt want to just put random code together without knowing what each part was doing, so i spent some time going through examples and understanding the basic structure.still a lot to figure out, especially how the motion capture system will eventually send position data to the drone, but this is a start.

![](https://fabricate.hackclub-assets.com/4919536a7b0d599ff855b2939cb12bdfc1d09cab1a2f95d776db75f2eccdb0fd/image-png/Screenshot%202026-11-13%20at%2011.55.34%E2%80%AFPM.png)

**Total time spent: 5h 54m**

# November 16, 2026: the final code

so this week i went into the backend code for the mocap side, mostly the kalman filter and the low pass filters. honestly i had no idea what a kalman filter was before this lol. i read a bunch of explanations and then went through alot of yt videos.The kalman filter keeps a guess of where the drone is, how fast it’s going, and how fast the speed is changing. then every new frame it guesses where the drone should be now, compares that to what the camera actually saw, and kinda blends the two together. the numbers in the matrices decide how much it trusts the model vs the camera, and i’m still figuring out what good numbers even look like :( .the low pass filters are easier to get. they smooth out the wobbly velocity and heading so the drone doesn’t get jerky commands.
there’s still a lot i don’t get, especially the matrix math and how the time step is calculated. i’m gonna keep learning shit and try testing it with fake data before i trust it on the real drones.

![](https://fabricate.hackclub-assets.com/58512b942975385732b85e3b8258960f605c9a724cd47da4c85a8c934c60ae94/image-png/Screenshot%202026-11-16%20at%2010.41.31%E2%80%AFPM.png)

**Total time spent: 6h 18m**
