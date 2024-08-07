# Introduction
[![View 3DSpaceTiling on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/96128-3dspacetiling) [![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=preethamam/3DSpaceTiling)

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

# Citation
3D space tiling code for creating the crops of point clouds and 3D objects is available to the public. If you use this code in your research, please use the following BibTeX entry to cite:
```bibtex
@PhdThesis{preetham2021vision,
author = {{Aghalaya Manjunatha}, Preetham},
title = {Vision-Based and Data-Driven Analytical and Experimental Studies into Condition Assessment and Change Detection of Evolving Civil, Mechanical and Aerospace Infrastructures},
school =  {University of Southern California},
year = 2021,
type = {Dissertations & Theses},
address = {3550 Trousdale Parkway Los Angeles, CA 90089},
month = {December},
note = {Condition assessment, Crack localization, Crack change detection, Synthetic crack generation, Sewer pipe condition assessment, Mechanical systems defect detection and quantification}
}
```
# Feedback
Please rate and provide feedback for the further improvements.
