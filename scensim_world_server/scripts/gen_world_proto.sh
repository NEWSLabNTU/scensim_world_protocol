#!/bin/bash
# Regenerate scensim_world_server/scensim_world.desc from proto/scensim_world.proto (roadmaps 037, 046).
# The Python side reads this descriptor set instead of generated _pb2 code (see
# scensim_world_server/protocol.py).
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
protoc --proto_path="$here/../proto" --descriptor_set_out="$here/scensim_world_server/scensim_world.desc" \
    "$here/../proto/scensim_world.proto"
echo "wrote $here/scensim_world_server/scensim_world.desc"
