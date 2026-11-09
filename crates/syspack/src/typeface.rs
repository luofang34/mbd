//! Typeface packs: a lettering face's bytes and the provenance and licence
//! it ships under, as hashed assets of a `TypefacePack` syspack. An
//! instrument pack pins the faces its dials are lettered in; vehicles never
//! pin a typeface.

use serde::{Deserialize, Serialize};

use crate::SyspackError;
use crate::manifest::validate::is_slug;

#[cfg(test)]
mod tests;

/// What a `TypefacePack` holds.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TypefaceSpec {
    /// The asset holding the face, in the renderer's typeface format.
    pub face_asset: String,
    /// The asset stating the face's provenance and licence.
    pub provenance_asset: String,
    /// SPDX licence expression, or a `LicenseRef-` naming the provenance.
    pub licence: String,
    /// The standards, drawings or fonts the face follows, as cited.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub upstream: Vec<UpstreamSource>,
}

/// One source a face follows.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UpstreamSource {
    /// What it is, such as `MS33558 (ASG), revision C`.
    pub name: String,
    /// Where it is published.
    pub url: String,
}

/// A pin of one typeface pack, by id, version and the SHA-256 of its
/// `syspack.json`, as a vehicle pins an instrument pack.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TypefacePin {
    /// The typeface pack's package id.
    pub id: String,
    /// Its package version.
    pub version: String,
    /// SHA-256 of its `syspack.json` bytes, hexadecimal.
    pub manifest_sha256: String,
}

fn shape(detail: &'static str) -> SyspackError {
    SyspackError::TypefacePackShape { detail }
}

impl TypefaceSpec {
    /// Checks the spec against the assets its pack declares.
    pub(crate) fn validate(&self, declared: impl Fn(&str) -> bool) -> Result<(), SyspackError> {
        if !declared(&self.face_asset) {
            return Err(shape("a typeface pack must declare its face asset"));
        }
        if !declared(&self.provenance_asset) {
            return Err(shape("a typeface pack must declare its provenance asset"));
        }
        if self.licence.trim().is_empty() {
            return Err(shape("a typeface pack must state its licence"));
        }
        if self
            .upstream
            .iter()
            .any(|s| s.name.trim().is_empty() || !s.url.starts_with("https://"))
        {
            return Err(shape("an upstream source needs a name and an https URL"));
        }
        Ok(())
    }
}

impl TypefacePin {
    /// Checks the pin is well formed.
    pub(crate) fn validate(&self) -> Result<(), SyspackError> {
        let hex = self.manifest_sha256.len() == 64
            && self
                .manifest_sha256
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase());
        if is_slug(&self.id) && !self.version.is_empty() && hex {
            Ok(())
        } else {
            Err(shape(
                "a typeface pin needs a slug id, a version and a SHA-256",
            ))
        }
    }
}
