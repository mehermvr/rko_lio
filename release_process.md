just a documentation of RKO LIO's release process for my own sake.

steps: 

1. checkout a new branch because catkin will be pushing some changes automatically. (name it bump version or something). Set this as well:
```bash
git branch --set-upstream-to=origin/branch_name
```

2.
```bash
catkin_generate_changelog
``` 
from the root

3. 
```bash
catkin_prepare_release -t v
```
by default will bump the minor version. `-t` is tag prefix, since we like v0.x.x names

4. merge the branch with a PR. there will be a v0.x.x tag on that branch on some commit. which is lost after the merge. so you'll have to retag current master, here's the commands
```bash
git checkout master
git pull origin master
git branch -D branch_you_made
git tag -d v0.x.x
git push origin :refs/tags/v0.x.x
git tag v0.x.x
git push origin v0.x.x
```

5. once that's done, we can do the bloom release. cd to the release repo (i imagine, the docs dont specify) 
```bash
bloom-release --rosdistro rolling rko_lio --pretend
# or if you're brave
bloom-release --rosdistro <distro> rko_lio
```
we don't need to do first time releases so i wont mention that here. if i ever need to do some edits, i'll add those commands here later. verify the version when you run the `--pretend`.
That opens a PR against ros/rosdistro. Will take a little bit to get accepted. but then the build farm will pick it up after. I still have to see if there is a faster way to iterate on the actual build farm build (even locally), then to actually bump the version every time.

6. We're not done. python release. go to the releases page, and draft a new release. pick the new tag. and generate changelog. and then release. now you're done. if the workflows dont break
