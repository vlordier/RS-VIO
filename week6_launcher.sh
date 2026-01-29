#!/bin/bash
# Week 6 Quick-Start Script
# Run this Monday morning to begin Week 6 execution

set -e

REPO_ROOT="/Users/vincent/Work/RS-VIO"
TOOLS_DIR="$REPO_ROOT/tools"
DATA_DIR="/tmp/student_training_data"
RESULTS_DIR="/tmp/student_training_results"

echo "╔════════════════════════════════════════════════════════════════════════════╗"
echo "║                   WEEK 6 EXECUTION LAUNCHER                               ║"
echo "║               Student Network Real Data Training Pipeline                 ║"
echo "╚════════════════════════════════════════════════════════════════════════════╝"
echo ""

# Function to print colored output
status() {
    echo "🔷 $1"
}

success() {
    echo "✅ $1"
}

error() {
    echo "❌ $1"
    exit 1
}

# 1. Pre-flight checks
status "Running pre-flight checks..."

if [ ! -f "$TOOLS_DIR/student_network.py" ]; then
    error "student_network.py not found"
fi
success "student_network.py present"

if [ ! -f "$TOOLS_DIR/train_student_network.py" ]; then
    error "train_student_network.py not found"
fi
success "train_student_network.py present"

if [ ! -f "$TOOLS_DIR/test_complete_pipeline.py" ]; then
    error "test_complete_pipeline.py not found"
fi
success "test_complete_pipeline.py present"

# 2. Verify Week 5 implementation
status "Verifying Week 5 implementation..."

cd "$TOOLS_DIR"
python3 test_complete_pipeline.py > /dev/null 2>&1 || error "Pipeline test failed"
success "6-stage pipeline validation complete (ALL STAGES PASSING)"

# 3. Check Python packages
status "Checking Python dependencies..."

python3 << 'EOF'
import sys
packages = ['torch', 'numpy', 'PIL', 'tqdm']
for pkg in packages:
    try:
        __import__(pkg)
    except ImportError:
        print(f"❌ Missing package: {pkg}")
        sys.exit(1)
print("✅ All required packages available")
EOF

# 4. Setup directories
status "Setting up Week 6 directories..."

mkdir -p "$DATA_DIR" && success "Data directory: $DATA_DIR"
mkdir -p "$RESULTS_DIR" && success "Results directory: $RESULTS_DIR"

# 5. Print Week 6 roadmap
echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo "WEEK 6 EXECUTION ROADMAP"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "📅 MONDAY:    Implement 7 Rust export points"
echo "             └─ Location: src/pipelines/vio/teacher_exporter.rs"
echo "             └─ Reference: $REPO_ROOT/STUDENT_NETWORK_INTEGRATION.md"
echo ""
echo "📅 TUESDAY:   Generate real dataset (TUM-VI room1, 600+ frames)"
echo "             └─ Command: cargo run --release --bin teacher_exporter ..."
echo "             └─ Validate: python3 tools/test_dataset.py /tmp/student_training_data/metadata.json"
echo ""
echo "📅 WEDNESDAY: Train network on real data (50 epochs)"
echo "             └─ Command: python3 tools/train_student_network.py /tmp/student_training_data/metadata.json"
echo "             └─ Expected: 30 min (GPU), 2 hours (CPU)"
echo ""
echo "📅 THURSDAY:  Benchmark inference speed vs RANSAC"
echo "             └─ GPU: <2ms per frame expected"
echo "             └─ CPU: 5-10ms per frame expected"
echo ""
echo "📅 FRIDAY:    VIO pipeline integration & real-time testing"
echo "             └─ Integration: Load model in VIO initialization"
echo "             └─ Test: Run end-to-end on room1 sequence"
echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# 6. Print next steps
echo "📋 NEXT STEPS"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
echo "1. Read documentation (Monday morning):"
echo "   • STUDENT_NETWORK_INTEGRATION.md  (Rust integration guide)"
echo "   • WEEK_6_EXECUTION_PLAN.md        (Monday execution plan)"
echo ""
echo "2. Begin Rust export implementation:"
echo "   • Implement 7 data export points"
echo "   • Test with 10-frame export"
echo "   • Validate JSON format"
echo ""
echo "3. Generate real dataset (Tuesday):"
echo "   • Run teacher_exporter on full room1 sequence"
echo "   • Validate all 600+ frames with test_dataset.py"
echo ""
echo "4. Train network (Wednesday):"
echo "   cd $REPO_ROOT
python3 tools/train_student_network.py \\
  /tmp/student_training_data/metadata.json \\
  --epochs 50 --batch-size 32 --device cuda \\
  --output-dir /tmp/student_training_results"
echo ""
echo "5. Benchmark & integrate (Thursday-Friday):"
echo "   • Measure inference speed"
echo "   • Compare with RANSAC baseline"
echo "   • Integrate into VIO pipeline"
echo ""
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""

# 7. Status report
echo "✨ WEEK 6 READY TO LAUNCH"
echo "═══════════════════════════════════════════════════════════════════════════"
echo ""
success "All pre-flight checks passed"
success "Week 5 implementation verified"
success "Directories prepared"
success "Python dependencies available"
echo ""
echo "Start Monday with: STUDENT_NETWORK_INTEGRATION.md"
echo ""
echo "Status: 🟢 GREEN - Ready to proceed"
echo ""
