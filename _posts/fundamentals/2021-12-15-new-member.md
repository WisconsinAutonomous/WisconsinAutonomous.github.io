---
title: New Member Guide
author: Aaron Young
date: 2021-12-15 14:39:00 -0600
categories: [Fundamentals, Setup]
tags: [tutorials, setup, non-technical]
---

Welcome to Wisconsin Autonomous! This is our Wiki website, which provides tutorials, resources, and generally helpful pages for our members to refer to. This page outlines two things: what our team is and how to get started.

## What is Wisconsin Autonomous?

Wisconsin Autonomous is a student organization at the University of Wisconsin-Madison focused on the development of autonomous vehicles. For new students, that may be a confusing sentence, so we'll break it down.

[Student organizations](https://win.wisc.edu/organizations) at UW-Madison are student-led clubs which focus on a specific topic, competition, or event. Wisconsin Autonomous is registered with the [College of Engineering](https://www.engr.wisc.edu/) and is one of four teams under the Automotive program umbrella.

Autonomous vehicles are an increasingly popular topic in modern technology. Basically, an autonomous vehicle is a car that is able to navigate _without_ the need for human intervention. Major companies are feeding loads of money into this industry, including companies like Google ([Waymo](https://waymo.com/)), Amazon ([Zoox](https://zoox.com/)), Apple, Tesla, and more. We are aiming to produce engineers and computer scientists that are ready to contribute to these types of companies. That's the ultimate goal of our organization.

### Brief Team History and Competitions

##### FSG

Our team was founded in 2018 as an extension of [Wisconsin Racing](wisconsinracing.org) to compete in [Formula Student Germany Driverless](https://spectrum.ieee.org/students-race-driverless-cars-in-germany-in-formula-student-competition), which is an autonomous racing competition for minature formula-style vehicles. Because of the difficulty in scale of this competition and it taking place in Germany, we transitioned away after about one year.

##### Indy Autonomous Challenge

In Fall 2019, we applied for acceptance in the [Indy Autonomous Challange](indyautonomouschallenge.com). This is autonomous racing competition for IndyLights vehicles, which are only a little smaller than F1 cars and can drive at speeds up to 180mph. An international competition, we were competing with the best-of-the-best in terms of Universities developing autonomous vehicle technology. After about 6 months in the competition and solid performances in multiple events, we couldn't put up the giant price tag necessary to purchase the vehicle (on the scale of $300,000). Therefore, similarly to the FSG Driverless competition, we decided to not move forward with this competition.

##### evGrand Prix

In Spring 2020, after realizing autonomous vehicle development is as hard as it is and hardware development should not be a major focus of our team, we decided to join the [evGrand Prix](evgrandprix.org/autonomous). This is a student competition where teams purchase a kit go-kart, which they must effectively convert from being human driven to being autonomous. Teams then compete to produce the fastest driving car on a given racetrack. We are currently competing in this event still, and the next competition date is May 19th and 20th, in Indiana.

##### AutoDrive Challenge II

The fourth and final competition we have been involved in is the [AutoDrive Challenge II](https://www.sae.org/attend/student-events/autodrive-challenge-series2/). It is a major, four year competition that's sponsored by General Motors (GM)and SAE, among others. It is an international competition with 10 total teams, of which were selected from over 80 applicants. The primary focus of the competition is to develop a fully autonomous urban driving vehicle capable of Level 4 autonomoy, as defined by SAE. GM provides a [Chevy Bolt EUV](https://www.chevrolet.com/electric/bolt-euv) to all teams and this is the main testbed for the competition.

### Team Structure

The Wisconsin Autonomous organization is split up into multiple subteams in order to organize projects and people to best suite our competitions. We currently have 8 subteams, each under Software, Hardware, or Operations. We welcome all majors!

Outlined below are the different subteams and their purpose.

#### Software

The software team is the largest group of the three umbrella subteams (software, hardware, and operations). This is also the primary component of each competition; you can't have an autonomous vehicle without the code which drives the car.

##### Perception

The perception team is tasked with visually understanding the environment around the vehicle. Think, for instance, how humans perceive the world around us. We use sensors such as cameras and [LiDARs](https://en.wikipedia.org/wiki/Lidar). This is a machine learning and computer vision heavy subteam.

##### State Estimation

State estimation is the team that attempts to calculate our position and velocity based on the sensors we have on the vehicle, and then map the surrounding environment. They utilize sensors such as GPS, [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit), and [encoders](https://en.wikipedia.org/wiki/Wheel_speed_sensor). They also utilize the results from [perception](#perception) and the calculate position to develop a 3D map. 

##### Driving Functions

Driving functions is the final stage of our control stack/logic. Using the perceived environment from [perception](#perception) and the generated state and map from [state estimation](#state-estimation), driving functions aim to actually control the vehicle through the world. Think of it like this: [perception](#perception) perceives the world, [state-estimation](#state-estimation) maps the world, then driving functions drives through it.

##### Software Platforms

Software platforms is the self described _IT_ group for Wisconsin Autonomous. We have many computers, sensors, and hardware that needs maintenance. Software platforms handles this, but developing software oriented solutions to ensure our vehicle runs reliably. Further, we have a simulation platform called [wa\_simulator](wa.wisc.edu/wa_simulator) that software platforms is tasked with maintaining.

#### Hardware

The hardware team ensures that our code has physical testbeds to actually run on. They work closely with the software team, and are an integral to the success of the team at our competition. 

##### Vehicle Systems

The vehicle systems subteam is responsible for vehicle level hardware projects and mounting solutions. They work to add actuators, mount our sensors/computers, and general vheicle maintenance. 

##### Vehicle Integration

Vehicle integration is a hardware and software oriented team. They work with the software teams and develop solutions to best integrate their code onto the vehicle. This includes wiring, testing of the software stack, and general advising for the software team. 

#### Operations

The operations team is focused on the business operations of Wisconsin Autonomous. You can think of Wisconsin Autonomous as a start-up; we need to acquire funding, employees (students), and successfully produce products. The operations team, though not exclusively, is made up of business majors that are interested in working with cutting edge technologies.

##### Marketing

The marketing team develops marketing solutions to publicize Wisconsin Autonomous and acquire sponsors. This includes adding content to our websites and developing style guides for emails, presentations, etc. 

##### Finances

Through our various competitions and sponsors, our team has a large fund for our technical engineering teams to use for parts and components. The finance team is tasked with managing this and ensuring we are best suited for years to come.

## Next Steps

What do you do now? To get involved, start with the following steps.

### Step 1 - Join the Slack Group

Separate post: [link](/posts/slack-group).

### Step 2 - Join the Microsoft Team

Separate post: [link](/posts/ms-team).

### Step 3 - Join the GitHub Organization 

_GitHub access is only necessary software focused members or leaders._

Separate post: [link](/posts/github-org).

### Step 4 - Join the Google Drive

Simply send a team leader your email and they will add you as a content manager. Make sure to use your wisc.edu email!
