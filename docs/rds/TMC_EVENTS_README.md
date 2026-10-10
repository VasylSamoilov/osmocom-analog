# TMC Event Code Table (tmc_events.csv)

Source: OpenStreetMap wiki TMC/Event_Code_List
URL: https://wiki.openstreetmap.org/wiki/TMC/Event_Code_List
License: Re-derivation of ISO 14819-2 event list; OSM community-maintained.
Extracted from: redsea src/tmc/events_data.cc (MIT license, windytan/redsea)
Row count: 1553 main events + 233 supplementary info rows = 1786 total

## CSV format (semicolon-delimited)

Header: Code;Description;Description with Q;N;Q;T;D;U;C;R

Columns:
  Code        - ISO 14819-2 event code (integer, 0-2047)
  Description - Human-readable event description
  Description with Q - Description with quantifier placeholder (Q)
  N           - Nature of event (not always populated)
  Q           - Quantifier type (0-12, or blank)
  T           - Duration type (D=direct, P=persistence)
  D           - Directionality (single/both/none)
  U           - Urgency (U=urgent, N=normal, X=extreme)
  C           - Update class
  R           - Cross-reference to related codes

## Usage in RDS TMC decoder

The 11-bit event code from group 8A (C[10:0]) can be looked up here
for human-readable descriptions. Code 0 = "unknown/unspecified".

## Related standards

- ISO 14819-1: Main TMC coding standard (free-format labels, DP table)
- ISO 14819-2: Event codes (this table)
- ISO 14819-3: Location codes (national tables, not included here)
- ISO 14819-6: Encrypted TMC
