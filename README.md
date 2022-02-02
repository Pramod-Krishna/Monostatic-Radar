# ***_Monostatic Radar_***

***

The design goal of this pulse radar system is to detect non-fluctuating targets with 1sqm RCS at a distance up to 5km from the radar with a range resolution of 50m. The desired performance index is a probability of detection (Pd) of 0.9 and probability of false alarm (Pfa) below 1e-6. 
We choose a rectangular waveform, and make a few assumptions:
* The only noise present at the receiver is the thermal noise, so there is no clutter involved in this simulation
* There are 3 stationary, non-fluctuating targets in space

## Output

![image](https://user-images.githubusercontent.com/54993262/151975346-7b23d312-6557-40d5-8b68-78a3b38b17a2.png)

# Improving the performance of Monostatic Radar

To improve the performance such that the maximum unambiguous range needs to be extended to 8 km and the system also needs to be able to detect Swerling case 2 targets.
The approach is to use Shnidman's equation, instead of Albersheim's equation, to calculate the required SNR. Due to the peak power being roughly eight times larger than the previous requirement, we should use appropriate signal processing techniques.

One way to reduce the power requirement is to use a waveform other than the rectangular waveform. For example, a linear FM waveform can use a longer pulse than a rectangular waveform. As a result, the required peak transmit power drops. Here the matched filter offers a processing gain which further improves the detection threshold. In addition, the added benefit of the matched filter of a linear FM waveform is that it compresses the waveform in the time domain so that the filtered pulse becomes much narrower, which translates to better range resolution. Then we integrate the receive pulses noncoherently to further improve the SNR.

Finally, threshold detection is performed on the integrated pulses. The detection scheme identifies the peaks and then translates their positions into the ranges of the targets.

![Improved_MR](https://user-images.githubusercontent.com/54993262/152088544-6e6f02d5-fae0-4bf8-ae33-edd2070d6e93.JPG)


## References 
* [Mathworks](https://in.mathworks.com/)
