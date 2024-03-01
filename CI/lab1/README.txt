This folder contains all files needed for Lab1:

- Reference image IMG_0691 in .tiff and .CR2 formats: the .tiff format is the one we feed as input to the pipeline. The .CR2 format 
 format is only used s reference for selecting pixels in manual balancing (Section 4).
- lab1.m: main Matlab file containing the full pipeline.
- interpolate_bilinear.m, interpolate_nn.m: interpolation functions corresponding to the bilinear and nearest neighbour methods, used in Section 3 of lab1.m
- IMG_0691.png, IMG_0691.jpeg: compressed image obtained at the output of the pipeline in .png and .jpeg formats.

< ----------------- lab1.m ----------------->

The code is divided into 8 sections corresponding to the 8 stages of the pipeline. You can run each Section sequentially or run the whole code. Each stage will display figures with intermediate results. Two of them (manual balancing and gamma correction) can be either be run with a preset of previously selected parameters, or with an interactive GUI that allows to adjust parameters in execution time. Pay attention to the Command Window where you will be prompted for Sections 4 and 7. In the case where several methods are performed (Sections 3, 4 and 5), we select only one of them at the end of the stage, and only this result is fed to the next stage of the pipeline.

Note: as stated in the assignment, intermediate results are mostly quite dark. These are the ones displayed in the final code, but you will find commented lines corresponding to intermediate brightened representations at different stages that were use for debug. This is specially relevant for the Denoising stage where the report shows the brightened representations, as it was quite difficult to compare results using the darker images.

Following is an overview Section and the figures it displays.

2.1 Reading the image: done automatically. Displays width, height and bit per pixel in the command window. 
	figure 1: original image in .TIFF format as read at the input of the pipeline
	figure 2: brightened version of Figure 1. 

2.2 Linearization: done automatically.
	figure 3: linearized image

3. Demosaicing: first we try the Matlab demoisaic() function which serves a baseline, then we perform both NN and bilinear interpolation and display the results. 

Bilinear interpolation is selected at the end to be fed to the next stage.

	figure 4: demosaiced image using Matlab function demosaic()
	figure 5: demoisaiced image using Nearest Neighbour
	figure 6: demosaiced image using Bilinear Interpolation

4. White balancing: grey world and white world are computed automatically. For manual balancing, you will be asked through the command window if you want to use a preset value (Y) or do it interactively (N). If you decide for the latter, you will see the .CR2 image displayed and a highlighted cursor over the image. Click on the object in the image you want to use as reference, and press Enter. After this, manual balancing will be computed based on your selected pixel value and you will see the results on Figure 9. After this, you will be asked through the Command Window if you are satisfied with the results (Y) and move on to the next stage, or if you want to try again (N). If you are not satisfied, the reference image with the cursor will be shown again, just select any pixel and repeat the process. 

The result of manual balancing is the one used for further stages.

	figure 7: grey world assumption
	figure 8: white world assumption
	figure 9: manual balancing result

5. Denoising: the three methods are computed automatically and displayed for you to compare.

The result of median denoising is the one used for the next stage.

	figure 10: mean filter denoising
	figure 11: median filter denoising
	figure 12: gaussian filter denoising

6. Color balance: performed automatically. A scale factor of 1.3 for the saturation channel is selected.

	figure 13: color balanced image after saturation boost

7. Tone reproduction: this stage can be done interactively or using the presets. When reaching this Section, Matlab will display the same
content of Figure 13 with two sliders below. The one above computes the scaling factor for brightness, the one below selects the gamma
parameter. Adjust these values and the Command Window will ask you to proceed (Y). DO NOT touch the sliders if you want to use the presets. After this, figure 15 will be displayed with the results of gamma correction according to the parameters selected. You will be asked if you want to proceed (Y) or try again (N). If you want to try again, re-adjust the sliders on Figure 14 and THEN press N. Figure 15 will refresh showing your new results.

	figure 14: same as figure 13, reference with sliders to select gamma correction parameters
	figure 15: gamma corrected image according to the parameters selected with the sliders in figure 14

8. Compression: a .png and a .jpeg image are generated, the latter according to the compression rate selected. We found 60% to be the minimal value where essential details are not lost, and so this is the preset. 
