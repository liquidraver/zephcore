# SPDX-License-Identifier: Apache-2.0
# Vendored ed25519 (orlp/ed25519) - no seed.c (ED25519_NO_SEED)
# Note: CMake warning about "not treated as a Zephyr library" is harmless - library still works.

zephyr_library_named(ed25519)
zephyr_library_sources(
	add_scalar.c
	fe.c
	ge.c
	key_exchange.c
	keypair.c
	sc.c
	sha512.c
	sign.c
	verify.c
)
zephyr_library_compile_definitions(ED25519_NO_SEED=1)
zephyr_library_include_directories(.)
