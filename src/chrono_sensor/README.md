# Chrono::Sensor feature showcase

A visual tour of what the Chrono::Sensor module can simulate: camera, lidar and radar sensors rendered with hardware ray tracing, covering camera output modes, lens models, the physical camera model, lighting and materials, range sensors, and multi-sensor rigs.

Each animation links to the **single self-contained demo** that produced it. They are ordinary Chrono demos, `src/demos/sensor/demo_SEN_showcase_*.cpp`, built with the rest of the demos, and use only assets that ship with Chrono, so they are reproducible from a stock checkout. The frames→WebP step and full instructions live in [`tools/`](tools/README.md).

The demos configure the scene exclusively through the public scene API and the standard sensor classes, so they build against whichever ray-tracing backend the configuration selected. The one exception is the fog demo: scene fog is part of the scene API on `ChOptixScene` but not on `ChVulkanRTScene`, so that demo is built only where the backend provides it. All parked-car demos share one canonical camera orbit, so their animations are frame-synchronized with each other and can be compared side by side.

> **How these were captured.** Every animation on this page came out of the demo it links to, run headlessly and saved through `ChFilterSave`, then encoded by the scripts in [`tools/`](tools/README.md). They were captured on one ray-tracing backend. Running the same demo on another will not reproduce them pixel for pixel: two GPU ray tracers draw different random numbers and carry different float and transcendental precision, so no two agree bit for bit. Treat the animations as a picture of what the module simulates, not as reference output to diff against.

---

## 📷 Camera modes

The camera sensor renders four synchronized output types from the same ray-traced scene.

| RGB | Depth |
|:---:|:---:|
| ![RGB](webp/camera_rgb.webp)<br>[▶ demo_SEN_showcase_camera_rgb.cpp](../../src/demos/sensor/demo_SEN_showcase_camera_rgb.cpp) | ![Depth](webp/camera_depth.webp)<br>[▶ demo_SEN_showcase_camera_depth.cpp](../../src/demos/sensor/demo_SEN_showcase_camera_depth.cpp) |
| Photorealistic hardware ray tracing: glossy PBR paint, HDR sky, soft shadows. | Per-pixel range: near surfaces bright, falling to black at the far clip. A ray that hits nothing returns the max depth, matching OptiX. |

| Surface normals | Segmentation |
|:---:|:---:|
| ![Normal](webp/camera_normal.webp)<br>[▶ demo_SEN_showcase_camera_normal.cpp](../../src/demos/sensor/demo_SEN_showcase_camera_normal.cpp) | ![Segmentation](webp/camera_segmentation.webp)<br>[▶ demo_SEN_showcase_camera_segmentation.cpp](../../src/demos/sensor/demo_SEN_showcase_camera_segmentation.cpp) |
| World-space surface normals encoded as RGB, revealing fine body-panel curvature. | Semantic + instance labels rendered directly by the ray tracer. |

---

## 🔭 Lenses & physical camera

Realistic optics and sensor imperfections, all computed in the ray generator / post chain.

| Fisheye (equidistant) | Radial (barrel) distortion |
|:---:|:---:|
| ![Fisheye](webp/lens_fisheye.webp)<br>[▶ demo_SEN_showcase_lens_fisheye.cpp](../../src/demos/sensor/demo_SEN_showcase_lens_fisheye.cpp) | ![Radial](webp/lens_radial.webp)<br>[▶ demo_SEN_showcase_lens_radial.cpp](../../src/demos/sensor/demo_SEN_showcase_lens_radial.cpp) |
| Wide-angle FOV lens (~143 deg): the horizon bends around the frame. | Action-cam barrel distortion via radial coefficients (k₁,k₂,k₃). |

| Depth of field | Sensor grain & vignette |
|:---:|:---:|
| ![Depth of field](webp/physcam_dof.webp)<br>[▶ demo_SEN_showcase_physcam_dof.cpp](../../src/demos/sensor/demo_SEN_showcase_physcam_dof.cpp) | ![Grain](webp/physcam_grain.webp)<br>[▶ demo_SEN_showcase_physcam_grain.cpp](../../src/demos/sensor/demo_SEN_showcase_physcam_grain.cpp) |
| Thin-lens bokeh from a 12 mm lens at f/2.0 focused at 6 m; the whole car sits inside the in-focus zone while the ground falls away. | The physical sensor model: cos⁴ lens vignetting darkening the corners, plus shot and read noise grain. |

---

## 💡 Lighting & materials

Physically based lighting: path-traced global illumination, HDR image-based lighting, soft area-light shadows, fog, and analytic lights.

| Global illumination | Area-light soft shadows |
|:---:|:---:|
| ![GI](webp/gi.webp)<br>[▶ demo_SEN_showcase_gi.cpp](../../src/demos/sensor/demo_SEN_showcase_gi.cpp) | ![Area lights](webp/arealights.webp)<br>[▶ demo_SEN_showcase_arealights.cpp](../../src/demos/sensor/demo_SEN_showcase_arealights.cpp) |
| Path-traced indirect bounce light fills the shadows (vs. flat ambient). | Disk + rectangle area lights cast true penumbra-edged soft shadows. |

| HDR environment reflections | Fog |
|:---:|:---:|
| ![Env map](webp/envmap.webp)<br>[▶ demo_SEN_showcase_envmap.cpp](../../src/demos/sensor/demo_SEN_showcase_envmap.cpp) | ![Fog](webp/fog.webp)<br>[▶ demo_SEN_showcase_fog.cpp](../../src/demos/sensor/demo_SEN_showcase_fog.cpp) |
| An HDR environment map is mirrored in the glossy paint and glass. | Exponential scattering fog fades the scene into distance haze. |

| Night, headlights on | |
|:---:|:---:|
| ![Night](webp/night_headlights.webp)<br>[▶ demo_SEN_showcase_night_headlights.cpp](../../src/demos/sensor/demo_SEN_showcase_night_headlights.cpp) | |
| Static rear view of a parked Audi in a near-black night: it sits dark, then the headlights switch on and two spot-light beams sweep the road ahead. | |

---

## 📡 Range sensors

Lidar and radar trace the **same accelerated BVH** as the cameras and return true range data. Here their raw returns are decoded into the representations engineers actually use.

| 360 deg lidar, bird's-eye point cloud | Forward radar, Doppler returns |
|:---:|:---:|
| ![Lidar](webp/lidar.webp)<br>[▶ demo_SEN_showcase_lidar.cpp](../../src/demos/sensor/demo_SEN_showcase_lidar.cpp) | ![Radar](webp/radar.webp)<br>[▶ demo_SEN_showcase_radar.cpp](../../src/demos/sensor/demo_SEN_showcase_radar.cpp) |
| A roof lidar sweeps a full circle as the car drives; returns are shown as a car-centric BEV point cloud, colored by height, with the obstacle lane and side walls clearly resolved. | A front radar's returns, placed in its forward FOV wedge and colored by Doppler closing speed: brighter and redder as the car drives toward the obstacle field. |

## 🛰️ Multi-sensor rig

Multiple sensors run **simultaneously** off one moving vehicle, sharing a single accelerated ray-traced scene.

| Four sensors at once (driving) |
|:---:|
| ![Multi-sensor](webp/multisensor.webp)<br>[▶ demo_SEN_showcase_multisensor.cpp](../../src/demos/sensor/demo_SEN_showcase_multisensor.cpp) |
| An Audi drives past semantically-labelled obstacles while an **RGB camera, depth camera, surface-normal camera, and segmentation camera** all sample the identical chase view every frame, shown here as a live 2x2 panel. |

---

*Simulation sources: [`src/demos/sensor/demo_SEN_showcase_*.cpp`](../demos/sensor) · post-processing and reproduction steps: [`tools/`](tools/README.md).*
