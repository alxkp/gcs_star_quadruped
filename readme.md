# GCS* Whole body Motion Planning with a Quadruped
- [ ] Implement GCS*
- [ ] Write Tests
- [ ] Property testing?
- [ ] Implement footstep planner
- [ ] Simulator and visualization
- [ ] Done!

# Installation

This package requires specific conda dependencies before installation. Set up your environment as follows:

```bash
# Create and activate conda environment
conda create -n gcs python=3.10 libstdcxx-ng>=12.2.0 -c conda-forge
conda activate gcs

# Install the package
pip install --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly .
```

Note: Drake requires Python 3.10/3.11 and a recent version of libstdcxx-ng.
We tested with 3.11 and libstdcxx-ng==14.2.0

