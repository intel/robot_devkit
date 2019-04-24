#!/bin/bash

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

function replace()
{
local repos=$1
local src_dir=$2
cd "$src_dir" || exit
for x in *
do
  cd "$x" || exit
  if [[ ${x} = "OpenCL" || ${x} = "Openvino" ]]
  then
    cd .. || exit
    echo "continue opencl or openvino"
    continue
  fi
  for y in *
  do
    cd "$y" || exit
    for repos_file in $(find "$repos" -name '*.repos')
    do
      number=$(sed -n "/$x\/$y:/"= "$repos_file")
      if test "${number}"
      then
        number=$((number+2))
        commitID=$(git rev-parse HEAD)
        sed -i "${number}a\    version: $commitID" "$repos_file"
        number=$((number+2))
        sed -i "${number}d" "$repos_file"
        echo "set" "${x}""/""${y}"" commit ID in""$repos_file"
        break
      else
        echo "could not find ""${x}""/""${y}"" in"" $repos_file"
      fi
    done
    cd ../ || exit
  done
  cd ../ || exit
done
}

function main()
{
echo "$CURRENT_DIR"

repos=${CURRENT_DIR}/../products/current/core
src_dir=${CURRENT_DIR}/../sdk_ws/core_ws/src/
echo "********start core ********"
replace "${repos}" "${src_dir}"
echo "********end   core ********"

repos=${CURRENT_DIR}/../products/current/device
src_dir=${CURRENT_DIR}/../sdk_ws/device_ws/src/
echo "********start device ********"
replace "${repos}" "${src_dir}"
echo "********end   device ********"

repos=${CURRENT_DIR}/../products/current/modules
src_dir=${CURRENT_DIR}/../sdk_ws/modules_ws/src/
echo "********start modules ********"
replace "${repos}" "${src_dir}"
echo "********end   modules ********"
}

main "$@"
