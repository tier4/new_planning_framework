#!/usr/bin/env python3
"""
Evaluate multiple LIVE trajectory runs from a configuration file
"""
import yaml
import json
import subprocess
import argparse
from pathlib import Path

def evaluate_all_runs(final_bag, input_bag, config_path, output_dir, map_path):
    """Evaluate all runs specified in config file"""

    # Load configuration
    with open(config_path) as f:
        config = yaml.safe_load(f)

    base_topic = config['base_trajectory_topic']
    runs = config['runs']

    print(f"Loaded config with {len(runs)} runs to evaluate")
    print(f"Base trajectory topic: {base_topic}\n")

    results = []
    for i, run_config in enumerate(runs, 1):
        prefix = run_config['prefix']
        description = run_config.get('description', prefix)

        # Build full topic name
        topic = f"/{prefix}{base_topic}"
        eval_output = Path(output_dir) / prefix
        eval_output.mkdir(parents=True, exist_ok=True)

        print(f"{'='*80}")
        print(f"[{i}/{len(runs)}] Evaluating: {description}")
        print(f"Prefix: {prefix}")
        print(f"Topic: {topic}")
        print(f"Output: {eval_output}")
        print(f"{'='*80}\n")

        # Build evaluation command
        script_dir = Path(__file__).parent
        eval_script = script_dir / "run_evaluation.sh"

        cmd = [
            "bash", str(eval_script),
            final_bag,
            input_bag,
            topic,
            str(eval_output),
        ]

        if map_path:
            cmd.append(map_path)

        # Run evaluation
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Evaluation failed for {prefix}: {e}")
            continue

        # Collect results
        result_json = eval_output / "or_results.json"
        if not result_json.exists():
            # Check if it was saved in home directory
            result_json = Path.home() / f"or_scene_evaluation_results_{prefix}.json"

        if result_json.exists():
            with open(result_json) as f:
                data = json.load(f)
                results.append({
                    "prefix": prefix,
                    "description": description,
                    "mean_ade": data["summary"]["ade"]["mean"],
                    "std_ade": data["summary"]["ade"]["std"],
                    "mean_fde": data["summary"]["fde"]["mean"],
                    "std_fde": data["summary"]["fde"]["std"],
                    "total_or_events": data["summary"]["total_or_events"],
                })
        else:
            print(f"Warning: Results JSON not found for {prefix}")

    # Generate comparison
    if results:
        generate_comparison(results, output_dir)
    else:
        print("No results collected!")

def generate_comparison(results, output_dir):
    """Generate comparison table and summary"""

    # Sort by ADE (best first)
    results.sort(key=lambda x: x["mean_ade"])

    # Print table
    print("\n" + "="*100)
    print("Multi-Run Comparison Results (Sorted by Mean ADE)")
    print("="*100)
    print(f"{'Description':<45} {'Prefix':<25} {'Mean ADE (m)':<20} {'Mean FDE (m)':<20}")
    print("-"*100)
    for r in results:
        print(f"{r['description']:<45} {r['prefix']:<25} {r['mean_ade']:>6.3f} ±{r['std_ade']:<6.3f}      {r['mean_fde']:>6.3f} ±{r['std_fde']:<6.3f}")
    print("="*100)

    # Calculate improvement
    best = results[0]
    worst = results[-1]
    improvement_pct = ((worst['mean_ade'] - best['mean_ade']) / worst['mean_ade'] * 100)

    print(f"\nBest Performing Run: {best['description']}")
    print(f"  Mean ADE: {best['mean_ade']:.3f}m (±{best['std_ade']:.3f}m)")
    print(f"  Mean FDE: {best['mean_fde']:.3f}m (±{best['std_fde']:.3f}m)")

    if len(results) > 1:
        print(f"\nWorst Performing Run: {worst['description']}")
        print(f"  Mean ADE: {worst['mean_ade']:.3f}m (±{worst['std_ade']:.3f}m)")
        print(f"\nImprovement (best vs worst): {improvement_pct:.1f}%")

    # Save to JSON
    summary = {
        "runs": results,
        "best_run": best,
        "worst_run": worst if len(results) > 1 else None,
        "improvement_percent": improvement_pct if len(results) > 1 else 0.0
    }

    summary_path = Path(output_dir) / "comparison_summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)

    print(f"\n✓ Comparison summary saved to: {summary_path}")

def main():
    parser = argparse.ArgumentParser(
        description='Evaluate multiple LIVE runs from configuration file'
    )
    parser.add_argument('--final-bag', required=True, help='Final bag with all LIVE trajectories')
    parser.add_argument('--input-bag', required=True, help='Original input bag')
    parser.add_argument('--config', required=True, help='YAML configuration file')
    parser.add_argument('--output-dir', required=True, help='Output directory')
    parser.add_argument('--map-path', default='', help='Optional map path')

    args = parser.parse_args()

    evaluate_all_runs(
        args.final_bag,
        args.input_bag,
        args.config,
        args.output_dir,
        args.map_path
    )

if __name__ == "__main__":
    main()
