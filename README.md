# Asymmetric Weapon Balancer for Fusion 360

A Fusion 360 add-in that balances asymmetric spinner weapons for combat robots. You tick the sketch dimensions it may change, and it adjusts them until the centre of mass of the weapon lies exactly on its spin axis, in both directions.

<p align="center">
  <img src="example.png" width="300" alt="Example 1">
  <img src="example2.png" width="300" alt="Example 2">
</p>

## What it does

You select the bodies that spin and point at the spin axis. The add-in then measures every sketch dimension (`d1`, `d2`, ...) and shows, right in the list, how far each one moves the centre of mass and how much of the offset it can fix. You decide which of them are acceptable to change in your design, and tick those. Everything else stays exactly as it is. It adjusts the ticked dimensions until the centre of mass sits on the axis, and leaves them at those values.

When more than two dimensions are ticked there are usually many balanced designs, so the add-in picks the one that changes your design the least: the smallest overall proportional change. Dimensions that don't affect the balance are left alone.

There are no naming conventions to follow. The weapon can be modelled any way you like, on any sketch plane, with pockets, fillets and chamfers. It can also be made of several bodies with different materials, such as a steel blade with a hardened bolt-on tooth.

## Requirements

- A parametric design (with a timeline).
- A dimensioned sketch. Every sketch dimension gets a name like `d1`, `d2`, `d3`. To see which name belongs to which dimension, open **Modify → Change Parameters**.
- Something that marks the spin axis: the bore face, a circular edge of the bore, a sketch circle, or a construction axis.

## Installation

1. Download the `AsymmetricWeaponBalancer` folder. Keep the folder name, the `.py` file name and the `.manifest` file name the same.
2. In Fusion 360, open **Utilities → Add-Ins → Scripts and Add-Ins**.
3. Click **+** and choose the `AsymmetricWeaponBalancer` folder.
4. Select it in the list and click **Run**. The add-in starts automatically from then on.

The **Balance Weapon** button appears in its own **ARCHMASTER** tab in the Design workspace toolbar, in the **Weapons** panel.

## Usage

1. Click **ARCHMASTER → Balance Weapon**.
2. **Weapon bodies**: select every body that spins with the weapon.
3. **Spin axis**: click the field, then select the bore face or edge, a sketch circle, or a construction axis.
4. Wait for the analysis. The add-in changes each dimension by 0.2 % in turn to measure its effect, with a progress bar you can cancel. This uses Fusion's preview, and every dimension is restored afterwards, so the design is not changed. It runs again only when you change the bodies or the axis, or click **Re-analyse**.
5. Read the summary. It lists every dimension that moves the centre of mass, strongest first, with what a +1 % change does in each direction and how much of the offset it can fix on its own. Dimensions with no effect, and dimensions that break the model, are listed separately.
6. **Dimensions to change**: tick the dimensions the add-in may adjust. Only you know which changes keep the weapon usable (tooth geometry, clearances, strength), so the add-in doesn't pick for you. Each entry shows its effect, for example `d7 = 69.111 mm · Weapon — 0.267 mm/%, alone fixes 61 % at -8.0 %` (see [Reading the dimension list](#reading-the-dimension-list)).
7. Below the list, a line tells you instantly whether the ticked dimensions can balance both directions and how much they would change.
8. Click **Balance**.

The report shows every dimension that changed (old value, new value and percentage), the remaining offset and imbalance, and the new mass. **Ctrl+Z** undoes the whole balance in one step. The next time you open the command in the same design, it remembers your bodies, axis, ticked dimensions and options.

## Options

- **Max change ± %** (default 45 %): how far each ticked dimension may move from its current value. Lengths never go to zero or below.

Dimensions defined by an expression, such as `d5 = d3 / 2` or `d7 = RadiusS`, are shown with their expression in the list. If you tick one, its expression is replaced by the solved number, and the readout warns you about it.

## Choosing dimensions

The centre-of-mass offset has two components, so you need at least two dimensions, and they must push the centre of mass in **different** directions. Two dimensions that both push it along the same line can only fix the offset along that line.

- A dimension that is symmetric about the tooth centreline, such as a counterweight radius or a symmetric pocket, only moves the centre of mass **along** the centreline.
- To move it **across** the centreline, pick something asymmetric: the tooth sweep angle, the hook of the tooth, or a pocket's angular position.
- A dimension that rotates the whole weapon about the axis only moves the centre of mass sideways around the axis. It can never reduce the offset.

## Reading the dimension list

- **`0.267 mm/%`**: how far the centre of mass moves when this dimension changes by 1 %. Bigger means more effect.
- **`alone fixes 61 % at -8.0 %`**: on its own, changing this dimension by -8 % removes 61 % of the current offset. The other 39 % lies across the direction this dimension pushes, and needs a second dimension.
- **`(over limit)`**: the change needed is beyond the **Max change** limit.
- **`no effect`**: this dimension doesn't move the centre of mass, for example the bore, or a dimension in another sketch.
- **`breaks the model`**: a small change makes a sketch or feature fail.

A good pair is usually one dimension with a high "fixes" share plus one that pushes sideways to it. A dimension can move the centre of mass exactly the right way and still ruin the weapon, for example by thinning the tooth or shrinking the counterweight until it's weak, so only tick dimensions you are happy to see change.

When you tick dimensions, the check below the list gives one of these verdicts:

- **Controllable in both X and Y.** Balance is possible. An estimate of the change needed is shown and compared with the max change limit.
- **Both, but weakly.** The dimensions push in nearly the same direction, so fixing the other direction needs large changes.
- **One direction only.** The offset across that line can't be removed. Add a dimension that moves the centre of mass sideways.

When you click **Balance**, the same check is applied. It reuses the analysis, so no extra rebuilds are needed if the design hasn't changed. If balance looks impossible or out of reach, the add-in shows the check and asks before changing anything.

## How it works

The analysis nudges every dimension by 0.2 % to measure how it moves the centre of mass. Dimensions defined by expressions are restored to their expressions between measurements, so links like `d5 = d3 / 2` keep working. From these measurements the add-in rates each dimension, finds the smallest pairs that cancel the offset, and checks the ticked set. With two or more dimensions it then steps towards the balanced design with the smallest proportional change, keeping every dimension within its limit and backing off when a value makes a sketch or feature fail. A typical weapon balances in 10 to 40 rebuilds.

With exactly one dimension it uses a scan and root finder instead. One dimension can only move the centre of mass along one line, so any offset across that line is reported rather than fixed.

## Reading the report

The offset is shown in both in-plane directions, named after the model axes so they match Fusion's **Properties** dialog. A weapon sketched on the XY plane spins about Z and is balanced in **X** and **Y**. One sketched on XZ spins about Y and is balanced in **X** and **Z**. The live readout in the dialog uses the same format.

- **Balanced in both X and Y.** The centre of mass is on the axis within 0.0001 mm in both directions.
- **Improved, but not fully balanced.** The ticked dimensions can't reach balance within their limits. The report lists which dimensions hit the limit, why the solve stopped, and the controllability check. Tick more dimensions or raise the max change.
- **No effect on balance** / **model fails to rebuild when changed.** Those dimensions were left unchanged.

If the solve fails or you cancel it, every dimension is restored to its original expression.

## Notes

- Leave unticked anything the rest of the robot depends on: bore and keyway, bolt pattern, outer diameter if it's set by ground clearance, and thickness.
- The report also shows the centre of mass position **along** the spin axis. That is normally mid-thickness, so Properties shows it as a non-zero value (for example Y = half the thickness for a weapon sketched on XZ). It doesn't cause imbalance, and the add-in leaves it alone.
- If the spin axis isn't at the model origin, Properties shows the centre of mass at the axis position rather than at zero. The offsets in the report are always measured from the spin axis.
- The selected bodies' materials are used, so assign materials before balancing multi-material weapons.
- Balancing changes the mass. The report shows the new mass so you can check it against your weight class.
- Solver settings (tolerance, rebuild limits, finite-difference step, toolbar tab) are at the top of `AsymmetricWeaponBalancer.py`.

## Purpose

The add-in was created to help design asymmetric spinner weapons for combat robot tournaments.

## Updates - sharajshreyas

- Improve root estimation in bracket method
- Update body comparison to use entityToken

## License

MIT - see [LICENSE](LICENSE). Free to use, modify and share.

If you use this add-in in a project, video or publication, a mention or a link back would be appreciated.
