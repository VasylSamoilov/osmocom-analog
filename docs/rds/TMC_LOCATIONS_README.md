# TMC Location Tables

## tmc_locations_italy.csv

Italian ALERT-C location code list (LCL v4.8) -- 5,911 road entries.
Full LCL: NAMES.DAT + ROADS.DAT join, from CCISS/MIT v4.8 (2024-02-22).

### Provenance

Converted from Database RDS-TMC 4.8 published by CCISS (Centro di
Coordinamento Informazioni sulla Sicurezza Stradale) on behalf of the
Italian Ministry of Infrastructure and Transport (MIT):
  https://www.cciss.it/web/cciss/database-rds-tmc

Source: NAMES.DAT + ROADS.DAT joined by RNID (same method as Germany).
Produces "ROADNUMBER - road_name" per LCD code (e.g. "A1", "SP12 - Bucine").
CID=25, TABCD=1, version 4.8, dated 22/02/2024. UTF-8.

Previous version: gr-rds tmc_locations_italy.h (GPL-2.0+, v2.2 NAMES-only,
13,194 entries). CCISS 4.8 has 5,911 road records with road numbers vs
13,194 name-only entries -- proper "A1" format rather than bare place names.

### License

The CCISS database is publicly distributed by MIT/CCISS with no login
required. No explicit redistribution license is bundled. The download page
states this is the official database for TIC/TCC data exchange and the
RAI RDS-TMC service. Attribution: CCISS/MIT, Database RDS-TMC v4.8.

### Format

Semicolon-delimited CSV, header row:
  LCD;Name

LCD is the 16-bit ALERT-C location code. Name is the Italian road or
junction name (autostrada number, city, interchange, etc.).

### Usage

Pass to the decoder via --rds-tmc-locations or call directly:

  rds_tmc_load_locations("docs/rds/tmc_locations_italy.csv");

The verbose 8A TMC log will then show:
  RDS 8A TMC: event=20 (stationary traffic) loc=10 (A1) dir=neg ...

This table covers Italian broadcasts only. For other countries you need
that country's national LCL (NAMES.DAT + ROADS.DAT from a standard LCL
export), which can be loaded directly as a directory:

  rds_tmc_load_locations("/path/to/national_lcl_directory/");

## Adding other national tables

Standard LCL directories (ISO 14819-3) contain NAMES.DAT and ROADS.DAT as
semicolon-delimited files. The loader joins them to produce
"ROADNUMBER - road_name" strings per LCD code. Pass the directory path.

Known open sources (verify licensing before use):
  - Germany: BAST (Federal Highway Research Institute) -- check bast.de
  - Norway: Datex-II national portal (registration required)
  - Netherlands: NDW uses VILD (incompatible format, not directly loadable)

## tmc_locations_germany.csv

German ALERT-C location code list (LCL 22.0) -- 4,185 road entries.

### Provenance

Converted from BASt LCL 22.0 (lcl-download.zip), published by
Bundesanstalt fuer Strassenwesen (BASt) on behalf of BMDV.
Landing page: https://www.bast.de/DE/Themen/Digitales/HF_1/Massnahmen/LCL/location-code-list.html

Conversion: NAMES.DAT + ROADS.DAT joined by RNID into LCD;Name CSV.
Only ROADS entries with a ROADNUMBER or resolved road name are included.

### License

Creative Commons Attribution 4.0 International (CC BY 4.0).
Attribution: Bundesanstalt fuer Strassenwesen (BASt) / BMDV, LCL 22.0, 2023.

### Coverage

Covers German motorways (Autobahnen), federal roads (Bundesstrassen),
Landstrassen, Staatsstrassen and urban roads.
