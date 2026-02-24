# M1 Tuning Tracks

This document locks the two mandatory SLAM tuning tracks from Addendum v1.1.

## Track 1: Open Area Loop

- Environment: `walls` world with wide open section.
- Flight profile:
  - speed target: 2.5-3.0 m/s
  - smooth loop with at least one long straight segment
  - duration: 3-5 minutes
- Pass criteria:
  - no SLAM reset/divergence
  - map remains consistent after loop closure revisits

## Track 2: Corridor Revisit Loop

- Environment: corridor-like path with repeated geometry.
- Flight profile:
  - speed target: 1.5-2.5 m/s
  - complete at least two revisits to same corridor
  - duration: 3-5 minutes
- Pass criteria:
  - loop closures occur without catastrophic false positives
  - relocalization remains stable after revisits
