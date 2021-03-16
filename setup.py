from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='benchbot_supervisor',
    version='2.1.0',
    author='Ben Talbot',
    author_email='b.talbot@qut.edu.au',
    description='The BenchBot Supervisor, used in the BenchBot Software Stack',
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=find_packages(),
    install_requires=['flask', 'gevent', 'pyyaml', 'requests'],
    classifiers=(
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
    ))
