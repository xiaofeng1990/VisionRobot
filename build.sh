#!/bin/bash
out_dir=output
mkdir -p ${out_dir}
function USAGE () {
    echo "./build.sh [arguments]" # arguments are accessible through $1, $2,...
    echo
    echo "Arguments:"
    echo "  -m: only make/make install"
    echo "  -h: help"
    echo
}

in_params=$@
only_make=
specify_param=
specify_targets=

for p in ${in_params} 
do
    case $p in
        -m)
            only_make=on
        ;;
        -h)
            USAGE
            exit 0
        ;;
        *)
            if [[ "$p" =~ ^-(v=.*)$ ]]; then
                specify_param=${p##*=}
            else
                specify_targets=$p
            fi
        ;;
    esac
done

pushd ${out_dir} > /dev/null

if [[ -z "$only_make" ]]; then
    cmake  ..
fi

make -j"$(nproc)" $specify_targets
make install/strip $specify_targets

popd > /dev/null