# ExoMy - Forked Software Repository

This repository contains the software to run Exomy. The [wiki](https://github.com/0xD0M1M0/ExoMy/wiki) explains you how to use it.

I forked the main repository of ESA-PRL and merged all my open pull requests into this new main. This comes after many talks with the ESA-Team at ESTEC stating that they cannot merge any unchecked code (which is understandable), but do not have any funding to work on this project.

So feel free to submit your issues and pull requests here. I will try my best to continue supporting this project and release software updates and bug fixes.

## UCL Modifications

We built and absolutely loved the ExoMy robot. Since UCL are providing the PanCam and instrument and also working on the Enfys instrument for the Rosalind Franklin rover, we wanted a slightly more representative mast head as part of our outreach activities. We decided to modify the original ExoMy design with the following enhancements:

- Modified mast head with a PanCam and Enfys instrument
- Mast head to support Pan and Tilt with the addition of 2 new motors
- Locomotion to be enhanced by adding an additional motor to each wheel to support wheel walking.

As per the original ExoMy project, the CAD model and software repository are to be separate. This repo contains only the updated software to support these new features.

### TODOs

- [ ] Test new configure script
- [ ] Review YAML output and determine how to track addr and bus
- [ ] Move new motor class to a more appropriate file
- [ ] Move pip to project folder
- [ ] Update picture in readme
- [ ] Update installation/configuration instructions

### Features to add

- [ ] Following reset, send halt commands to everything
- [ ] When jockstick returns to 0 also send stop command.
- [ ] When looses connection with WIFI also stop.
  
![ExoMy image](https://github.com/0xD0M1M0/ExoMy/wiki/images/renderings/2020_02_25.JPG)

# ExoMy Project Structure

### [Website](https://esa-prl.github.io/ExoMy/)

There is a website about ExoMy. It does not help you build it, but is still nice to look at.

### [Wiki](https://github.com/0xD0M1M0/ExoMy/wiki)

The [wiki](https://github.com/0xD0M1M0/ExoMy/wiki) contains step by step instructions on how the 3D-printed rover ExoMy can be built, controlled and customized.

### [Documentation Repository](https://github.com/0xD0M1M0/ExoMy)

This repository contains all the files of the documentation of ExoMy. Just click on [*Releases*](https://github.com/esa-prl/ExoMy/releases) and download the files of the latest release. They are explained further in the [wiki](https://github.com/0xD0M1M0/ExoMy/wiki).

### Social Media
<!-- Add icon library -->
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.13.1/css/all.css">

<!-- Add font awesome icons -->
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/discord-brands.svg" width="20px">
    <a href="https://discord.gg/gZk62gg"> Join the Community!</a>  
</p>
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/twitter-square-brands.svg" width="20px">
    <a href="https://twitter.com/exomy_rover"> @ExoMy_Rover</a>
</p>
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/instagram-square-brands.svg" width="20px">
    <a href="https://www.instagram.com/exomy_rover/"> @ExoMy_Rover</a>
</p>
