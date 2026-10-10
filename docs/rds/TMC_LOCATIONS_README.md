# TMC Location Tables

## tmc_locations_italy.csv

Italian ALERT-C location code list (LCL) -- 13,194 entries, LCD codes 0..13445.

### Provenance

Converted from `tmc_locations_italy.h` in gr-rds (GNU Radio):
  https://github.com/bastibl/gr-rds/blob/master/lib/tmc_locations_italy.h

The gr-rds file was built from the official Italian LCL v2.2 (DAT_22.zip)
published by il portale dell'automobilista (Italian MCTC). Only the NAMES.dat
portion (LCD -> location name) was carried over; ROADS/SEGMENTS/POINTS were
discarded.

### License

The gr-rds file carries: Copyright 2004 Free Software Foundation, Inc.,
GNU General Public License v2 or later. The underlying Italian location data
has separate terms from the original Italian source.
osmocom-analog is GPL-licensed; this file is compatible.

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
