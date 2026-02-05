{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix/monthly";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { nixpkgs, flake-utils, fenix, ... }: flake-utils.lib.eachDefaultSystem (system:
    let pkgs = import nixpkgs { inherit system; };
      libraries = with pkgs; [
        wayland
        libxkbcommon
        vulkan-loader
        udev
        alsa-lib
        openssl
      ];
    in {
      devShells.default = pkgs.mkShell {
        buildInputs = [
          (with fenix.packages.${system}; combine [
            latest.toolchain
            latest.rustc-codegen-cranelift-preview
          ])
          pkgs.dioxus-cli
          pkgs.pkg-config
          pkgs.mold
          pkgs.clang
        ] ++ libraries;
        LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath libraries;
      };
    }
  );
}
