FROM julia:latest

WORKDIR /root

RUN julia -e 'using Pkg; Pkg.add(url="https://github.com/kjellqvist/MinimaxAdaptiveControl.jl"); Pkg.add("JuMP","Hypatia");'

# Install Git
RUN apt update
RUN apt --yes install git

# Clone Exploration Repo
RUN git clone https://github.com/kwesiRutledge/OzayGroupExploration

