This folder contains all files needed for Lab2:

- Reference image "burano" in .jpe

Scripts:

deblurring_demo.m: This script visualizes the results of applying Wiener, Lucy-Richardson, and Wiener without prior deconvolutions to grayscale images using the selected aperture. Each method generates its own figure for comparison.

color_deblurring_demo.m: Similar to deblurring_demo.m, this script demonstrates the results of the deconvolution methods applied to color images using the specified aperture. Each method is displayed in a separate figure for evaluation.

apertures_deblurring_demo.m: This script presents the outcomes of applying Wiener, Lucy-Richardson, and Wiener without prior deconvolutions to grayscale images using all available apertures. The results are visualized together in one figure to facilitate comparison.

power_spect_demo.m: This script illustrates the frequency effects of the chosen aperture through visualizations. It provides insights into how the aperture impacts frequency characteristics.

compare_power_spect.m: Similar to power_spect_demo.m, this script compares the frequency effects of all apertures in a single plot. It offers a comprehensive view of how each aperture influences frequency responses.

explore_sigma_blurSize.m: This script explores the correlation between blurSize and sigma values. It showcases the effects of each value on the image and generates heatmaps of PSNR and SSIM plots for further analysis.