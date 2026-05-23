from setuptools import setup, find_packages

with open("README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="pylibnmea2k",
    version="0.2.4",
    description="Minimal, fast NMEA 2000 single-frame PGN decoder",
    long_description=long_description,
    long_description_content_type="text/markdown",
    license="MIT",
    python_requires=">=3.10",
    packages=find_packages(),
    keywords=["nmea2000", "n2k", "marine", "sailing", "can", "pgn"],
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "Topic :: Scientific/Engineering",
        "Topic :: System :: Networking",
    ],
    project_urls={
        "Repository": "https://github.com/ghotihook/pylibnmea2k",
    },
)
