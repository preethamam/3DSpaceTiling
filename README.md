# Introduction
Divides the 3D space with or without overlaps. Tested on 3D point clouds.

# Quick Pipeline Visualization
## Example 1: PCA based re-orientation
| Tiles  | PCA plane | XYZ plane views |
| ------------- | ------------- | ------------- |
| ![tiles](https://user-images.githubusercontent.com/28588878/126412827-74ba8010-4206-4511-b967-8334eade1bff.png)  | ![fitPlane](https://user-images.githubusercontent.com/28588878/126412858-4d5d8a06-2882-4f38-bf2a-befea5c2a28c.png)  | ![planeViews](https://user-images.githubusercontent.com/28588878/126412838-2ab6aa8a-1744-493a-8032-33e0b4a2253f.png) |

## Example 2: Projection based re-orientation example (very accurate)
#### Requires MatGeom toolbox
| Original point cloud  | Original point cloud XYZ planes | Re-oriented XYZ plane views |
| ------------- | ------------- | ------------- |
| ![tilesEngine](https://user-images.githubusercontent.com/28588878/127075812-c2996924-7541-4cab-ab05-df48e0c8affa.png) | <img width="863" alt="Screen Shot 2021-07-23 at 4 39 07 AM" src="https://user-images.githubusercontent.com/28588878/126776720-28232c94-2537-4f3e-9e9e-425d39643b4a.png"> | <img width="874" alt="Screen Shot 2021-07-23 at 4 39 23 AM" src="https://user-images.githubusercontent.com/28588878/126776740-376f342a-43cf-47d9-9d8e-a9fd595b07ae.png"> |

# Requirements
MATLAB <br />
MATLAB - Computer Vision Toolbox <br />
[MatGeom toolbox](http://github.com/mattools/matGeom)

# MATLAB Central
[![View 3DSpaceTiling on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/96128-3dspacetiling)
