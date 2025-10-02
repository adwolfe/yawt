# Pull Request Template

Thank you for contributing to YAWT! Please fill out the sections below so reviewers can quickly understand the intent, scope, and safety of this change.

## Summary
<!-- What changed and why? Keep this brief but specific. -->
- 

## Linked Issues
<!-- Use "Fixes #<id>" to auto-close, or "Refs #<id>" to link. -->
Fixes #
Refs #

## Type of Change
<!-- Select all that apply. -->
- [ ] Bug fix
- [ ] Feature
- [ ] Refactor
- [ ] Performance
- [ ] Build/CI
- [ ] Docs
- [ ] Chore

## Affected Areas
<!-- Select all that apply (used for labels and reviewers). -->
- [ ] core
- [ ] gui
- [ ] data
- [ ] utils
- [ ] build
- [ ] ci
- [ ] docs

## Detailed Changes
<!-- Bullet list of notable code changes, new modules, file moves, etc. -->
- 
- 
- 

## Screenshots / Logs (optional)
<!-- Attach images/log excerpts that help reviewers understand the change. -->
- 

## How to Test
<!-- Provide exact steps and commands to verify locally. Include inputs and expected outcomes. -->
Environment prerequisites:
- macOS version:
- Qt 6 version:
- OpenCV version:
- Compiler:

Build (Debug):
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(qtpaths --install-prefix)
make -j"$(sysctl -n hw.logicalcpu)"
open -n yawt.app --args --debug-mode
```

Test steps:
1) 
2) 
3) 

Expected results:
- 

## Risk and Rollback
Risk level: [low | medium | high]  
Rollback plan: [revert PR | feature flag | guarded code-path]  
User impact if rolled back: 

## Performance / Memory / Threading Considerations
<!-- Call out any changes to CPU/memory usage, algorithmic complexity, or threading model. -->
- Parallelism/threading changes:
- Cross-thread signals/slots introduced/changed:
- cv::Mat copying / memory lifetime considerations:
- Estimated memory delta on typical videos:

## Breaking Changes
<!-- List API changes, behavior changes, or migration steps for users or downstream code. -->
- 

Migration notes:
- 

## Documentation
- [ ] Updated CLAUDE.md if developer/agent workflow changed
- [ ] Updated README/ROADMAP for user-facing or planning changes
- [ ] N/A

## Tests
- [ ] Added/updated unit tests
- [ ] Added integration/smoke tests
- [ ] Manually verified core flows
- [ ] N/A

## Safety Checklist
- [ ] CI passes (build + any tests)
- [ ] No long-running or blocking scripts (build steps terminate)
- [ ] QObject parent/child ownership verified for new QObjects
- [ ] New types used in queued signals are Q_DECLARE_METATYPE + qRegisterMetaType
- [ ] No unapproved changes to algorithmic constants/heuristics (e.g., PHYSICAL_BLOB_IOU_THRESHOLD)
- [ ] OpenCV/Qt discovery remains configurable (no hard-coded local paths)
- [ ] App launches locally after build (Debug/Release as applicable)

## Release Notes (optional)
<!-- One-line entry suitable for a CHANGELOG. -->
- 

## Additional Notes
<!-- Anything else reviewers should know (trade-offs, follow-ups, todos). -->
- 
