# PGP-DOR
PGP-DOR: A Point-Grid-Point Scheme for Efficient Dynamic Object Removal

## Overview
We propose PGP-DOR, a novel method for accurately identifying and efficiently removing dynamic objects in autonomous driving maps. Our approach leverages a Point-Grid-Point (PGP) update strategy to fully exploit the spatiotemporal attributes of BEV grids while distinguishing motion attributes at both the point cloud and BEV levels. Additionally, we employ Bayesian Generative Kernel Inference (BGKI) for dense dynamic attribute inference, significantly improving the accuracy and robustness of dynamic object removal. Experimental results on public datasets and self-collected data show that PGP-DOR outperforms state-of-the-art methods in both online and offline scenarios.

<div align="center">
  
| Traversability Model | Pointcloud Result | BEV Result |
| ------- | ------- | ------- |
| ![](assets/traversability_model.gif) | ![](assets/pointcloud_result.gif) | ![](assets/BEV_result.gif) |
<!-- | ------- | ------- | ------- | -->

### Comparison with State-of-the-Art  
Our method achieves superior dynamic object removal compared to existing approaches.  
![](assets/img/qualitative_evaluation.png)

</div>

## Repository Status
Currently, the supporting code and dataset are under review as part of the publication process. The materials will be made publicly available upon acceptance of the paper. Please stay tuned for updates.

## Citation
If you find our work helpful, please consider citing our paper (to be updated upon acceptance).

## Contact
For any inquiries, feel free to reach out via GitHub issues or email.
