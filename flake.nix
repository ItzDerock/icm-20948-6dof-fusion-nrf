{
  description = "Embassy nRF52840 dev environment (nice!nano UF2)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      fenix,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        target = "thumbv7em-none-eabihf";

        toolchain = fenix.packages.${system}.combine [
          fenix.packages.${system}.latest.rustc
          fenix.packages.${system}.latest.cargo
          fenix.packages.${system}.latest.clippy
          fenix.packages.${system}.latest.rustfmt
          fenix.packages.${system}.latest.rust-src
          fenix.packages.${system}.latest.llvm-tools-preview
          fenix.packages.${system}.targets.${target}.latest.rust-std
        ];

        uf2conv = pkgs.stdenvNoCC.mkDerivation {
          name = "uf2conv";
          src = pkgs.fetchFromGitHub {
            owner = "microsoft";
            repo = "uf2";
            rev = "master";
            sha256 = "sha256-KurHemymW8d3HchA3TckUS96cDCNeyr0giDmDpkQjz8=";
          };
          installPhase = ''
            mkdir -p $out/bin
            cp utils/uf2conv.py $out/bin/uf2conv
            cp utils/uf2families.json $out/bin/
            chmod +x $out/bin/uf2conv
          '';
        };

        flash = pkgs.writeShellScriptBin "flash" ''
          set -euo pipefail
          BIN=''${1:?Usage: flash <binary-name>}
          MOUNT=''${UF2_MOUNT:-/run/media/$USER/NICENANO/}
          WORK=$(mktemp -d)
          trap 'rm -rf "$WORK"' EXIT

          echo ":: Building $BIN (release)..."
          cargo build --release --bin "$BIN"

          echo ":: Converting to UF2..."
          ${pkgs.cargo-binutils}/bin/rust-objcopy \
            target/${target}/release/"$BIN" \
            -O ihex "$WORK"/firmware.hex
          ${pkgs.python3}/bin/python3 ${uf2conv}/bin/uf2conv \
            "$WORK"/firmware.hex \
            --family 0xADA52840 \
            --convert \
            --output "$WORK"/firmware.uf2

          echo ":: Waiting for $MOUNT ..."
          while [ ! -d "$MOUNT" ]; do sleep 0.5; done

          echo ":: Flashing..."
          cp "$WORK"/firmware.uf2 "$MOUNT"/
          echo ":: Done! Device will reboot."
        '';
      in
      {
        devShells.default = pkgs.mkShell {
          name = "embassy-nrf52840";

          buildInputs = [
            toolchain
            flash
            pkgs.cargo-binutils
            pkgs.cargo-expand
            pkgs.flip-link
            (pkgs.python3.withPackages (
              ps: with ps; [
                pyserial
                numpy
                pygame
                pyopengl
              ]
            ))
          ];

          CARGO_BUILD_TARGET = target;
          DEFMT_LOG = "debug";
        };
      }
    );
}
