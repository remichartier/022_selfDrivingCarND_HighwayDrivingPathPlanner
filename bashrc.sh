git config --global user.email "remipr.chartier@gmail.com"
git config --global user.name "remichartier"
git remote set-url origin "git@github.com:remichartier/022_selfDrivingCarND_HighwayDrivingPathPlanner.git"
apt install openssh-client
eval "$(ssh-agent -s)"
ssh-add .ssh/20201224_1219_github_ed25519