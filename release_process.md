just a documentation of RKO LIO's release process for my own sake.

# subsequent releases

steps: 

1. checkout a new branch because catkin will be pushing some changes automatically. (name it bump version or something)
```bash
export new_branch=branch_name
export new_version=version_number_without_v
git checkout -b $new_branch
git push
git branch --set-upstream-to=origin/$new_branch
```

check Changelog.rst and commit it

2. from the root
```bash
catkin_generate_changelog
vim CHANGELOG.rst
git add -p
git commit
message -> "changelog"
git push
``` 

3. 
```bash
catkin_prepare_release -t v
```
by default will bump the minor version. `-t` is tag prefix, since we like v0.x.x names

4. modify pyproject to match version

```bash
vim python/pyproject.toml
modify the version
git add -p
git commit
git push
MERGE THE PR
```

5. merge the branch with a PR. there will be a v0.x.x tag on that branch on some commit. which is lost after the merge. so you'll have to retag current master, here's the commands
```bash
git checkout master
git pull origin master
git branch -D $new_branch
git tag -d v$new_version
git push origin :refs/tags/v$new_version
git tag v$new_version
git push origin v$new_version
```

6. once that's done, we can do the bloom release. cd to the release repo (i imagine, the docs dont specify) 
```bash
z rko_lio-release
bloom-release --rosdistro rolling rko_lio --pretend
# or if you're brave
bloom-release --rosdistro rolling rko_lio
bloom-release --rosdistro jazzy rko_lio
bloom-release --rosdistro kilted rko_lio
bloom-release --rosdistro humble rko_lio
```

7. python release. go to the releases page, and draft a new release. pick the new tag. and generate changelog. and then release. now you're done. if the workflows dont break

and we're done.

we don't need to do first time releases so i wont mention that here. if i ever need to do some edits, i'll add those commands here later. verify the version when you run the `--pretend`.
The above opens a PR against ros/rosdistro. Will take a little bit to get accepted. but then the build farm will pick it up after. I still have to see if there is a faster way to iterate on the actual build farm build (even locally), then to actually bump the version every time.

# new release

```bash
bloom-release --new-track --rosdistro jazzy --track jazzy rko_lio
bloom-release --new-track --rosdistro kilted --track kilted rko_lio
bloom-release --new-track --rosdistro humble --track kilted rko_lio
```

and default values for all entries because we already had a rolling track which configured everything.
