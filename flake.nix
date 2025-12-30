{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix";
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
          fenix.packages.${system}.latest.toolchain
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
