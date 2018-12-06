#ifndef MESH_SERDESER_H
#define MESH_SERDESER_H

int mesh_lib_serialize_state(const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint8_t *msg_buf,
                             size_t msg_len,
                             size_t *msg_used);

int mesh_lib_deserialize_state(struct mesh_generic_state *current,
                               struct mesh_generic_state *target,
                               int *has_target,
                               mesh_generic_state_t kind,
                               const uint8_t *msg_buf,
                               size_t msg_len);

int mesh_lib_serialize_request(const struct mesh_generic_request *req,
                               uint8_t *msg_buf,
                               size_t msg_len,
                               size_t *msg_used);

int mesh_lib_deserialize_request(struct mesh_generic_request *req,
                                 mesh_generic_request_t kind,
                                 const uint8_t *msg_buf,
                                 size_t msg_len);

#endif // MESH_SERDESER_H
