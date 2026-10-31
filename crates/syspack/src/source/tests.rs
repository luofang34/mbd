use std::collections::BTreeMap;

use super::*;

#[test]
fn a_memory_source_serves_what_it_holds() {
    let assets = BTreeMap::from([("a.bin".to_owned(), vec![1, 2, 3])]);
    let source = MemoryPackageSource::new(b"{}".to_vec(), assets);
    assert_eq!(source.read_manifest_bytes().unwrap(), b"{}");
    assert_eq!(source.read_asset_bytes("a.bin").unwrap(), [1, 2, 3]);
    assert!(matches!(
        source.read_asset_bytes("b.bin"),
        Err(SyspackError::AssetMissing { path }) if path == "b.bin"
    ));
}

#[test]
fn embedded_and_received_packages_are_the_same_source() {
    let received = MemoryPackageSource::new(
        b"{}".to_vec(),
        BTreeMap::from([("a.bin".to_owned(), vec![1, 2, 3])]),
    );
    let embedded = MemoryPackageSource::from_embedded(b"{}", &[("a.bin", &[1, 2, 3])]);
    assert_eq!(embedded, received);
}

#[test]
fn a_directory_serves_its_manifest_and_nested_assets() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir(dir.path().join("fdm")).unwrap();
    std::fs::write(dir.path().join(MANIFEST_FILE), b"{}").unwrap();
    std::fs::write(dir.path().join("fdm/a.xml"), b"<a/>").unwrap();
    let source = DirectoryPackageSource::new(dir.path());
    assert_eq!(source.read_manifest_bytes().unwrap(), b"{}");
    assert_eq!(source.read_asset_bytes("fdm/a.xml").unwrap(), b"<a/>");
}

#[test]
fn a_directory_reports_what_is_missing() {
    let dir = tempfile::tempdir().unwrap();
    let source = DirectoryPackageSource::new(dir.path());
    assert!(matches!(
        source.read_manifest_bytes(),
        Err(SyspackError::ManifestRead { .. })
    ));
    assert!(matches!(
        source.read_asset_bytes("fdm/a.xml"),
        Err(SyspackError::AssetMissing { .. })
    ));
}

#[test]
fn a_directory_refuses_to_climb_out_on_its_own() {
    let outer = tempfile::tempdir().unwrap();
    let package = outer.path().join("package");
    std::fs::create_dir(&package).unwrap();
    std::fs::write(outer.path().join("secret"), b"secret").unwrap();
    let source = DirectoryPackageSource::new(package);
    assert!(matches!(
        source.read_asset_bytes("../secret"),
        Err(SyspackError::UnsafeAssetPath { .. })
    ));
}

#[cfg(unix)]
#[test]
fn a_directory_refuses_symlinks_out_of_the_package() {
    let outer = tempfile::tempdir().unwrap();
    let package = outer.path().join("package");
    std::fs::create_dir(&package).unwrap();
    std::fs::write(outer.path().join("secret"), b"secret").unwrap();
    std::os::unix::fs::symlink(outer.path().join("secret"), package.join("file")).unwrap();
    std::os::unix::fs::symlink(outer.path(), package.join("dir")).unwrap();
    let source = DirectoryPackageSource::new(package);
    for path in ["file", "dir/secret"] {
        assert!(
            matches!(
                source.read_asset_bytes(path),
                Err(SyspackError::UnsafeAssetPath { .. })
            ),
            "{path}"
        );
    }
}
